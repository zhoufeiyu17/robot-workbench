"""Pure CPU target mesh rasterization; no device, camera or process lifecycle."""
import numpy as np


class MeshRenderer:
    """FK and mesh rasterization only; never calls geometry.preview()."""
    def __init__(self,geometry):
        self.geometry=geometry
        self.view=dict(yaw=.4,pitch=.35,zoom=1.,pan_y=0.)
        self.center=None;self.distance=1.8
        self.pose=None;self.surfaces=[];self.top_image=None;self.output_key=None;self.output=None

    def set_overview_view(self,view):self.view=dict(view)

    def render(self,q,gaps):
        import cv2
        pose=(tuple(q),tuple(gaps));output_key=(pose,tuple(sorted(self.view.items())))
        if output_key==self.output_key:return self.output
        if pose!=self.pose:
            geometry=self.geometry.with_gaps(*gaps)
            frames=geometry.frames(q);parts=[]
            for side in ('left','right'):
                for name,vertices,faces in getattr(geometry,'full_side_meshes',geometry.side_meshes)[side]:
                    transform=frames[side][name]
                    points=vertices[faces]@transform[:3,:3].T+transform[:3,3]
                    parts.append(points)
            for _,vertices,faces in getattr(geometry,'static_meshes',[]):parts.append(vertices[faces])
            if self.center is None:
                low=np.min([points.min(axis=(0,1))for points in parts],axis=0)
                high=np.max([points.max(axis=(0,1))for points in parts],axis=0)
                self.center=(low+high)/2;self.distance=max(.5,float(np.linalg.norm(high-low))*1.6)
            light=np.array([.5,-.4,.8]);light/=np.linalg.norm(light)
            self.surfaces=[]
            for triangles in parts:
                normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
                shade=np.clip(145+65*(normal@light)/np.maximum(np.linalg.norm(normal,axis=1),1e-12),90,210).astype(int)
                self.surfaces.append((triangles,normal,shade))
            self.pose=pose;self.top_image=None
        def draw(view,title):
            yaw,pitch=view['yaw'],view['pitch']
            outward=np.array([np.cos(yaw)*np.cos(pitch),np.sin(yaw)*np.cos(pitch),np.sin(pitch)])
            right=np.cross([0,0,1],outward);right/=np.linalg.norm(right)
            up=np.cross(outward,right);camera=self.center+outward*self.distance/view['zoom']
            basis=np.column_stack((right,-up,-outward));polygons=[]
            image=np.full((480,640,3),255,np.uint8)
            for triangles,normal,shade in self.surfaces:
                optical=(triangles-camera)@basis
                visible=np.all(optical[:,:,2]>.01,axis=1)&(np.einsum('ij,ij->i',normal,camera-triangles[:,0])>0)
                optical=optical[visible]
                uv=optical[:,:,:2]/optical[:,:,2,None]*520+[320,260+view.get('pan_y',0)*160]
                polygons.extend(zip(optical[:,:,2].mean(axis=1),np.clip(uv,-32768,32768).astype(np.int32),[(int(v),)*3 for v in shade[visible]]))
            for _,uv,color in sorted(polygons,key=lambda row:row[0],reverse=True):cv2.fillConvexPoly(image,uv,color)
            cv2.putText(image,title+' | display only',(12,23),cv2.FONT_HERSHEY_SIMPLEX,.5,(70,70,70),1,cv2.LINE_AA)
            return image
        image=draw(self.view,'3D VIEW')
        if self.view.get('top_view'):
            if self.top_image is None:self.top_image=draw(dict(yaw=np.pi,pitch=np.pi/2,zoom=1.,pan_y=0.),'TOP VIEW')
            image=np.hstack((image,self.top_image));image[:,639:641]=220
        ok,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,88])
        if not ok:raise ValueError('CPU target JPEG failed')
        self.output_key=output_key;self.output=(encoded.tobytes(),image)
        return self.output
