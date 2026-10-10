"""Rebind reviewed A1 geometry evidence only across optical leaf-origin changes."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

GEOMETRY_ID='44c6160a0a33a78710c228c5af2b1f9e364d8dc808f3330500830b79a2dc3b16'
OPTICAL=tuple(prefix+'_camera_rgb_optical_frame_joint' for prefix in ('top','left_wrist','right_wrist'))
LINKS=tuple(f'{side}_link{i+offset}' for side,offset in (('left',0),('right',10)) for i in (2,3,5))
ALGORITHMS=('a1_support_contact.py','a1_support_kernel.py','a1_support_triangles.py','a1_support_surface_bvh.py')
CERTIFICATES=('a1_conservative_refinement.json','a1_self_domain_certificate.json','a1_grasp_contact.json',
              'a1_support_domain.json','a1_support_near.json','a1_fixed_domain.json')
PROOF='a1-calibration-derivation.json'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def encoded(value):return (json.dumps(value,indent=2,allow_nan=False)+'\n').encode()
def value_sha(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def mesh_path(urdf,filename):
    urdf=Path(urdf).resolve()
    root,relative=(urdf.parents[1],filename.removeprefix('package://acone/')) if filename.startswith('package://acone/') else (urdf.parent,filename)
    if '://' in relative or Path(relative).is_absolute() or '..' in Path(relative).parts:
        raise ValueError('mesh must remain inside its authored asset root')
    path=(root/relative).resolve(strict=True)
    if not path.is_file() or not path.is_relative_to(root):raise ValueError('mesh escapes authored asset root')
    return path


def mechanical_identity(urdf):
    """Hash all authored structure and mesh bytes, omitting only 3 empty leaf poses."""
    urdf=Path(urdf);tree=ET.parse(urdf).getroot();joints=tree.findall('joint');links=tree.findall('link')
    by_joint={e.get('name'):e for e in joints};by_link={e.get('name'):e for e in links}
    if len(by_joint)!=len(joints) or len(by_link)!=len(links):raise ValueError('duplicate A1 node identity')
    for name in OPTICAL:
        joint=by_joint.get(name);child=name.removesuffix('_joint');parent=child.removesuffix('_rgb_optical_frame')+'_link'
        if (joint is None or joint.get('type')!='fixed' or joint.find('parent').get('link')!=parent
                or joint.find('child').get('link')!=child or child not in by_link or len(by_link[child])
                or any(e.find('parent').get('link')==child for e in joints)):
            raise ValueError('only the three geometry-free optical fixed leaf origins may change')
        origins=joint.findall('origin')
        if (len(origins)!=1 or set(origins[0].attrib)!={'xyz','rpy'} or len(origins[0])
                or (origins[0].text or '').strip()):
            raise ValueError('one plain xyz/rpy optical origin required')
    mesh_hashes={}
    def node(element):
        attrs=dict(element.attrib)
        for key in ('xyz','rpy','scale'):
            if key in attrs:
                values=[float(x) for x in attrs[key].split()]
                if len(values)!=3 or not all(math.isfinite(v) for v in values):raise ValueError('finite URDF vector required')
                attrs[key]=[v+0. for v in values]
        if element.tag=='mesh':
            path=mesh_path(urdf,attrs['filename'])
            if path not in mesh_hashes:mesh_hashes[path]=sha(path)
            attrs['filename_sha256']=mesh_hashes[path];del attrs['filename']
        children=[]
        for child in element:
            signature=node(child)  # Validate even the omitted optical pose.
            if element.tag=='joint' and element.get('name') in OPTICAL and child.tag=='origin':continue
            children.append(signature)
        return [element.tag,attrs,(element.text or '').strip(),children]
    return value_sha(node(tree))


def support_identity(urdf,native_code):
    """CPU equivalent of the existing explicit sidecar/hash gate; no geometry exemptions added."""
    urdf=Path(urdf);path=urdf.with_name('a1_support_contact.json');data=json.loads(path.read_bytes());policy=data['policy']
    if (data.get('schema')!='a1_local_support.v1' or data.get('urdf_sha256')!=sha(urdf)
            or policy.get('asset_geometry_id')!=GEOMETRY_ID or policy.get('max_interference_m')!=.0012
            or policy.get('separation_direction_link2')!=[0.,0.,1.] or policy.get('front_support_x_min_m')!=-.1
            or policy.get('claw_support_component_seed_faces')!={'link2':37470,'link5':14791}):
        raise ValueError('unreviewed or mismatched baseline A1 support policy/URDF')
    meshes={}
    for link in ET.parse(urdf).getroot().findall('link'):
        if link.get('name') not in LINKS:continue
        visuals=link.findall('visual')
        if len(visuals)!=1 or visuals[0].find('geometry/mesh') is None:raise ValueError('one support mesh per link required')
        meshes[link.get('name')]=sha(mesh_path(urdf,visuals[0].find('geometry/mesh').get('filename')))
    if set(meshes)!=set(LINKS) or meshes!=data.get('mesh_sha256'):raise ValueError('baseline support mesh identity mismatch')
    algorithms=Path(native_code)/'source/robot_sim/robot_sim/tasks/manager_based/coffee_make'
    return dict(sidecar_sha256=sha(path),sidecar_path=str(path),asset_geometry_id=GEOMETRY_ID,
        urdf_sha256=sha(urdf),mesh_sha256=meshes,algorithm_sha256={name:sha(algorithms/name) for name in ALGORITHMS},policy=policy)


def derive(source_urdf,bundle_urdf,target_urdf,native_code,certificate_root=None):
    """Prepare new bytes only after the baseline and protected mechanics agree."""
    source_urdf,bundle_urdf,target_urdf=map(Path,(source_urdf,bundle_urdf,target_urdf))
    identity=support_identity(source_urdf,native_code)
    original=mechanical_identity(source_urdf)
    if mechanical_identity(bundle_urdf)!=original:raise ValueError('A1 mechanical geometry/joint chain changed; optical-only rebinding refused')
    target_sha=sha(bundle_urdf)
    sidecar=dict(schema='a1_local_support.v1',policy=deepcopy(identity['policy']),
        urdf_sha256=target_sha,mesh_sha256=deepcopy(identity['mesh_sha256']))
    sidecar_bytes=encoded(sidecar)
    derived_identity=dict(identity,sidecar_path=str(target_urdf.with_name('a1_support_contact.json')),
        sidecar_sha256=hashlib.sha256(sidecar_bytes).hexdigest(),urdf_sha256=target_sha)
    provenance=dict(source_urdf_path=str(source_urdf.resolve()),source_urdf_sha256=identity['urdf_sha256'],derived_urdf_sha256=target_sha,
        source_sidecar_sha256=identity['sidecar_sha256'],derived_sidecar_sha256=derived_identity['sidecar_sha256'],
        mechanical_identity_sha256=original,allowed_origin_joints=list(OPTICAL))
    root=Path(certificate_root or Path(__file__).parent);files={};pins={};parts=set()
    for name in CERTIFICATES:
        path=root/name;data=json.loads(path.read_bytes())
        if data.get('urdf_sha256')!=identity['urdf_sha256']:raise ValueError('baseline certificate URDF differs: '+name)
        if 'source_parts_sha256' in data:parts.add(data['source_parts_sha256'])
        if 'support_identity' in data:
            if {k:v for k,v in data['support_identity'].items() if k!='sidecar_path'}!={k:v for k,v in identity.items() if k!='sidecar_path'}:
                raise ValueError('baseline support certificate identity/policy differs: '+name)
            data['support_identity']=deepcopy(derived_identity)
        source_sha=sha(path);data['urdf_sha256']=target_sha
        data['calibration_derivation']=dict(provenance,source_certificate_sha256=source_sha)
        files[name]=encoded(data);pins[name]=dict(source_sha256=source_sha,derived_sha256=hashlib.sha256(files[name]).hexdigest())
    if len(parts)!=1 or any(not isinstance(v,str) or len(v)!=64 for v in parts):
        raise ValueError('baseline mechanical certificate part identities differ')
    proof=dict(schema='a1_optical_derivation.v1',**provenance,source_parts_sha256=next(iter(parts)),
        source_support_identity=identity,derived_support_identity=derived_identity,certificates=pins)
    return sidecar_bytes,files,proof


def certificate_path(name,urdf):
    """Explicit asset-local derived proofs; unmarked old assets retain old paths."""
    default=Path(__file__).with_name(name)
    if name not in CERTIFICATES:raise ValueError('unknown A1 mechanical certificate')
    urdf=Path(urdf).resolve();path=urdf.with_name(PROOF)
    if not path.exists():return default
    if path.is_symlink():raise ValueError('regular asset-local derivation proof required')
    proof=json.loads(path.read_bytes())
    if (proof.get('schema')!='a1_optical_derivation.v1' or proof.get('derived_urdf_sha256')!=sha(urdf)
            or proof.get('derived_sidecar_sha256')!=sha(urdf.with_name('a1_support_contact.json'))
            or proof.get('allowed_origin_joints')!=list(OPTICAL)):
        raise ValueError('derived mechanical certificate asset identity differs')
    result=urdf.parent/'derived-certificates'/name
    if result.is_symlink() or result.parent.is_symlink() or result.resolve().parent!=result.parent:
        raise ValueError('derived certificate escapes its asset directory')
    if sha(result)!=proof['certificates'][name]['derived_sha256']:raise ValueError('derived certificate SHA differs: '+name)
    return result


def derive_cup_reference(reference,urdf,*,certificate_root=None):
    """Keep the accepted task pose; rebind only a proven optical-only URDF edit."""
    urdf=Path(urdf).resolve();path=urdf.with_name(PROOF)
    if path.is_symlink() or not path.is_file():raise ValueError('regular asset-local cup-reference derivation proof required')
    proof=json.loads(path.read_bytes())
    sidecar=Path(proof['source_support_identity']['sidecar_path'])
    # Older optical proofs name the reviewed native asset's sidecar rather than
    # its URDF. That asset has this fixed, already-reviewed entry point.
    source=Path(proof.get('source_urdf_path') or sidecar.with_name('acone_no_handle_d405.urdf'))
    if (source.resolve().parent!=sidecar.resolve().parent or sha(source)!=reference.get('urdf_sha256')
            or proof.get('source_urdf_sha256')!=sha(source) or proof.get('source_sidecar_sha256')!=sha(sidecar)):
        raise ValueError('cup-reference source differs from the accepted A1 baseline')
    expected_sidecar=dict(json.loads(sidecar.read_bytes()),urdf_sha256=sha(urdf))
    if json.loads(urdf.with_name('a1_support_contact.json').read_bytes())!=expected_sidecar:
        raise ValueError('cup-reference derived support policy/mesh differs from baseline')
    mechanical=mechanical_identity(source)
    if proof.get('mechanical_identity_sha256')!=mechanical or mechanical_identity(urdf)!=mechanical:
        raise ValueError('cup-reference mechanical geometry/joint chain changed')
    root=Path(certificate_root or Path(__file__).parent)
    for name in CERTIFICATES:
        original=root/name
        if (proof['certificates'][name]['source_sha256']!=sha(original)
                or json.loads(original.read_bytes()).get('urdf_sha256')!=reference['urdf_sha256']):
            raise ValueError('cup-reference baseline certificate differs: '+name)
        certificate_path(name,urdf)
    return dict(deepcopy(reference),urdf_sha256=sha(urdf),calibration_derivation=dict(
        proof_sha256=sha(path),source_reference_sha256=value_sha(reference),
        source_urdf_sha256=reference['urdf_sha256'],derived_urdf_sha256=sha(urdf),
        mechanical_identity_sha256=mechanical,allowed_origin_joints=list(OPTICAL)))
