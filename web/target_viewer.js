/* Display-only native model FK and GPU scene. Never sends control or camera requests. */
import * as THREE from './vendor/three.module.js';
import {OrbitControls} from './vendor/OrbitControls.js';

export function modelFrames(model,q,gaps){
  const limits=model.jaw_limits_mm||[[0,83],[0,83]];
  if(limits.length!==2||limits.some(pair=>!Array.isArray(pair)||pair.length!==2||!pair.every(Number.isFinite)||pair[0]!==0||pair[1]<=0||pair[1]>200))throw Error('invalid display jaw limits');
  if(q.length!==12||gaps.length!==2||!q.every(Number.isFinite)||!gaps.every((v,i)=>Number.isFinite(v)&&v>=limits[i][0]&&v<=limits[i][1]))throw Error('12 finite target joints and device-bounded jaw gaps required');
  const values=new Map(model.joints.map(row=>[row.name,0]));model.q_names.forEach((name,i)=>values.set(name,q[i]));
  for(const row of model.gap_joints)values.set(row.name,gaps[row.side]*row.scale+row.offset);
  const pending=new Map(Object.entries(model.mimics));
  for(let i=0;pending.size&&i<model.joints.length;i++)for(const [name,[parent,multiplier,offset]]of pending)if(values.has(parent)&&!pending.has(parent)){values.set(name,values.get(parent)*multiplier+offset);pending.delete(name);}
  if(pending.size)throw Error('unresolved display mimic dependency');
  const frames=new Map([[model.root,new THREE.Matrix4()]]);
  for(const row of model.joints){
    const parent=frames.get(row.parent);if(!parent)throw Error('display joint tree is not ordered');
    const axis=new THREE.Vector3(...row.axis),value=values.get(row.name),delta=new THREE.Matrix4();
    if(row.kind==='revolute'||row.kind==='continuous')delta.makeRotationAxis(axis,value);
    else if(row.kind==='prismatic')delta.makeTranslation(...axis.multiplyScalar(value).toArray());
    const origin=new THREE.Matrix4().fromArray(row.origin.flat()).transpose();frames.set(row.child,parent.clone().multiply(origin).multiply(delta));
  }
  const base=frames.get(model.base);if(!base)throw Error('display base is missing');
  const inverse=base.clone().invert();return new Map([...frames].map(([name,T])=>[name,inverse.clone().multiply(T)]));
}

export async function createTargetViewer({container,modelRef,request,signal,onFrame}){
  const response=await request(modelRef.url,{signal,cache:'no-store'});if(!response.ok)throw Error('GPU model manifest '+response.status);
  const model=await response.json();
  if(model.schema!=='workbench.target_model.v1'||model.model_id!==modelRef.model_id||!model.display_only||model.q_names?.length!==12||model.joints?.length>256
      ||!Number.isInteger(model.binary_bytes)||model.binary_bytes<=0||model.binary_bytes>64*1024*1024)throw Error('GPU display model identity or size differs');
  const binary=await request(modelRef.binary_url,{signal,cache:'no-store'});if(!binary.ok)throw Error('GPU model buffer '+binary.status);
  if(binary.headers.get('X-Model-SHA256')!==model.binary_sha256)throw Error('GPU model digest differs');
  const buffer=await binary.arrayBuffer();if(buffer.byteLength!==model.binary_bytes||buffer.byteLength%4)throw Error('GPU model byte count differs');
  if(signal.aborted)throw Error('GPU display context was replaced');
  const initialGaps=(model.jaw_limits_mm||[[0,83],[0,83]]).map(pair=>pair[1]);
  modelFrames(model,Array(12).fill(0),initialGaps);
  const renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,powerPreference:'high-performance'});
  renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));renderer.setClearColor(0xffffff);renderer.domElement.draggable=false;
  renderer.domElement.setAttribute('aria-label','GPU 本地目标姿态预览，不代表实际运动');renderer.domElement.dataset.modelId=model.model_id;
  const gl=renderer.getContext(),debug=gl.getExtension('WEBGL_debug_renderer_info');
  renderer.domElement.dataset.gpuRenderer=gl.getParameter(debug?debug.UNMASKED_RENDERER_WEBGL:gl.RENDERER);
  const scene=new THREE.Scene(),material=new THREE.MeshLambertMaterial({color:0x9b9fa5});scene.add(new THREE.AmbientLight(0xffffff,1.6));
  const light=new THREE.DirectionalLight(0xffffff,2.);light.position.set(2,-3,5);scene.add(light);
  const meshes=[];
  for(const row of model.meshes){
    if(!Number.isInteger(row.count)||row.count<=0||row.count%3||![row.offset,row.normal_offset].every(v=>Number.isInteger(v)&&v>=0&&v%4===0)||row.normal_offset+row.count*12>buffer.byteLength||row.offset+row.count*12>buffer.byteLength)throw Error('GPU mesh buffer range differs');
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(buffer,row.offset,row.count*3),3));geometry.setAttribute('normal',new THREE.BufferAttribute(new Float32Array(buffer,row.normal_offset,row.count*3),3));
    const mesh=new THREE.Mesh(geometry,material);mesh.matrixAutoUpdate=false;scene.add(mesh);meshes.push({mesh,link:row.link});
  }
  const low=new THREE.Vector3(...model.bounds[0]),high=new THREE.Vector3(...model.bounds[1]),center=low.clone().add(high).multiplyScalar(.5),distance=Math.max(.5,low.distanceTo(high)*1.6);
  const camera=new THREE.PerspectiveCamera(40,4/3,.005,50);camera.up.set(0,0,1);camera.position.copy(center).add(new THREE.Vector3(.9,.4,.35).normalize().multiplyScalar(distance));camera.lookAt(center);
  const top=new THREE.PerspectiveCamera(40,4/3,.005,50);top.up.set(1,0,0);top.position.copy(center).add(new THREE.Vector3(0,0,distance));top.lookAt(center);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target.copy(center);controls.enableDamping=false;controls.minDistance=.2;controls.maxDistance=10.;controls.update();
  let closed=false,frame=null,split=false,lastInput=performance.now(),frames=0,lastWidth=0,lastHeight=0;
  const render=()=>{
    frame=null;if(closed)return;
    const width=Math.max(1,container.clientWidth),height=Math.max(1,container.clientHeight),left=split?Math.floor(width/2):width;
    if(width!==lastWidth||height!==lastHeight){renderer.setSize(width,height,false);lastWidth=width;lastHeight=height;}renderer.setScissorTest(true);
    camera.aspect=left/height;camera.updateProjectionMatrix();renderer.setViewport(0,0,left,height);renderer.setScissor(0,0,left,height);renderer.render(scene,camera);
    if(split){top.aspect=(width-left)/height;top.updateProjectionMatrix();renderer.setViewport(left,0,width-left,height);renderer.setScissor(left,0,width-left,height);renderer.render(scene,top);}
    renderer.domElement.dataset.frame=String(++frames);renderer.domElement.dataset.inputDelayMs=(performance.now()-lastInput).toFixed(2);
    onFrame?.({frames,model_id:model.model_id,local:true,split});
  };
  const schedule=()=>{lastInput=performance.now();if(frame===null&&!closed)frame=requestAnimationFrame(render);};controls.addEventListener('change',schedule);
  const observer=new ResizeObserver(schedule);observer.observe(container);
  renderer.domElement.addEventListener('webglcontextlost',event=>{event.preventDefault();renderer.domElement.dataset.contextLost='true';});
  renderer.domElement.addEventListener('pointerdown',event=>{if(split&&event.offsetX>renderer.domElement.clientWidth/2)controls.enabled=false;},true);
  for(const event of ['pointerup','pointercancel'])renderer.domElement.addEventListener(event,()=>{controls.enabled=true;});
  container.append(renderer.domElement);
  const viewer={canvas:renderer.domElement,
    setPose(q,gaps){const matrices=modelFrames(model,q,gaps);for(const row of meshes)if(row.link){const T=matrices.get(row.link);if(!T)throw Error('GPU mesh link missing');row.mesh.matrix.copy(T);}schedule();},
    setView(view){split=view.top_view===true;const yaw=view.yaw,pitch=view.pitch;camera.position.copy(center).add(new THREE.Vector3(Math.cos(yaw)*Math.cos(pitch),Math.sin(yaw)*Math.cos(pitch),Math.sin(pitch)).multiplyScalar(distance/view.zoom));camera.lookAt(center);controls.target.copy(center);controls.update();schedule();},
    setTop(enabled){split=enabled;schedule();},
    destroy(){closed=true;if(frame!==null)cancelAnimationFrame(frame);observer.disconnect();controls.dispose();for(const row of meshes)row.mesh.geometry.dispose();material.dispose();renderer.dispose();renderer.domElement.remove();}
  };
  viewer.setPose(Array(12).fill(0),initialGaps);return viewer;
}
