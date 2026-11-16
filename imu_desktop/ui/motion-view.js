// Display-only mannequin. The measurement engine and exported angles are untouched.
import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/GLTFLoader.js';
import {OrbitControls} from 'three/addons/OrbitControls.js';

const $=id=>document.getElementById(id),mount=$('motionMount'),status=$('motionModelStatus');
const paths={full:new URL('./models/fullbody3.glb',import.meta.url).href,lower:new URL('./models/lower_body1.glb',import.meta.url).href};
let renderer=null,scene,camera,controls,model=null,animation=0,token=0,asset='',view='focus';
let modelId=$('model').value,side=$('side').value,latest=new Map();
const bones=new Map(),loader=new GLTFLoader();

function disposeModel(value){value?.traverse(o=>{o.geometry?.dispose();for(const m of Array.isArray(o.material)?o.material:o.material?[o.material]:[])m.dispose();});}
function frame(){
 if(!model)return;
 const box=new THREE.Box3().setFromObject(model),size=box.getSize(new THREE.Vector3());
 const height=size.y||1,focus=view==='focus'&&modelId==='upper_limb_v1';
 const y=box.min.y+height*(focus ? .68 : .5),distance=height*(focus ? 1.05 : 1.65);
 controls.target.set(0,y,0);camera.position.set(distance*.65,y+distance*.12,distance);
 camera.near=.01;camera.far=100;camera.updateProjectionMatrix();controls.update();
}
function pose(){
 if(!model)return;
 for(const bone of bones.values())bone.quaternion.copy(bone.userData.rest);
 const rotate=(name,angles)=>{const bone=bones.get(name);if(!bone||!angles.every(Number.isFinite))return;const e=new THREE.Euler(...angles.map(THREE.MathUtils.degToRad),'XYZ');bone.quaternion.copy(bone.userData.rest).multiply(new THREE.Quaternion().setFromEuler(e));};
 if(modelId==='upper_limb_v1'){
  const suffix=side==='left'?'.L':'.R',mirror=side==='left'?-1:1;
  const shoulder=latest.get('shoulder'),elbow=latest.get('elbow'),wrist=latest.get('wrist');
  // Local mannequin axes approximate the named movement for visual feedback only.
  if(shoulder?.measured){const a=shoulder.angles;if([a.shoulder_plane,a.shoulder_elevation,a.shoulder_axial].every(Number.isFinite))rotate('upper_arm'+suffix,[a.shoulder_elevation,mirror*a.shoulder_axial,mirror*a.shoulder_plane]);}
  if(elbow?.measured)rotate('forearm'+suffix,[elbow.angles.elbow_flexion,mirror*elbow.angles.elbow_supination,0]);
  if(wrist?.measured)rotate('hand'+suffix,[wrist.angles.wrist_flexion,0,mirror*wrist.angles.wrist_radial]);
 }else{
  for(const [joint,bone,sign]of [['hip','thigh.L',-1],['knee','shin.L',1],['ankle','foot.L',1]]){const r=latest.get(joint);if(r?.measured)rotate(bone,[sign*r.angles[joint+'_flexion'],0,0]);}
 }
 model.updateMatrixWorld(true);
}
function load(){
 if(!renderer)return;
 const name=view==='full'||modelId==='upper_limb_v1'?'full':'lower';
 $('motionModelTitle').textContent=view==='full'?'Mô hình toàn thân':modelId==='upper_limb_v1'?'Mô hình thân trên':'Mô hình thân dưới';
 if(asset===name&&model){frame();pose();return;}
 asset=name;const id=++token;status.textContent='Đang tải mô hình…';
 loader.load(paths[name],gltf=>{
  if(id!==token){disposeModel(gltf.scene);return;}
  if(model){scene.remove(model);disposeModel(model);}bones.clear();model=gltf.scene;
  model.traverse(o=>{if(o.isSkinnedMesh){o.frustumCulled=false;o.normalizeSkinWeights();o.skeleton.pose();for(const b of o.skeleton.bones){b.userData.rest=b.quaternion.clone();bones.set(b.name,b);}}else if(o.isMesh)o.visible=false;});
  model.rotation.y=Math.PI;model.updateMatrixWorld(true);
  let box=new THREE.Box3().setFromObject(model);model.scale.setScalar(1.8/(box.max.y-box.min.y||1));model.updateMatrixWorld(true);
  box=new THREE.Box3().setFromObject(model);const center=box.getCenter(new THREE.Vector3());model.position.set(-center.x,-box.min.y,-center.z);scene.add(model);model.updateMatrixWorld(true);
  status.textContent='Mô hình minh họa';frame();pose();
 },undefined,()=>{if(id===token){asset='';status.textContent='Không tải được mô hình 3D';}});
}

window.motionView={update(rows,id,selectedSide){latest=rows;const changed=modelId!==id||side!==selectedSide;modelId=id;side=selectedSide;if(changed)load();pose();}};
try{
 scene=new THREE.Scene();scene.background=new THREE.Color(0xdce8f7);
 camera=new THREE.PerspectiveCamera(45,1,.01,100);renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));renderer.outputColorSpace=THREE.SRGBColorSpace;mount.appendChild(renderer.domElement);
 scene.add(new THREE.HemisphereLight(0xffffff,0x5d6b78,1.3));const light=new THREE.DirectionalLight(0xffffff,1.2);light.position.set(2,4,3);scene.add(light);scene.add(new THREE.GridHelper(4,20,0x91a0af,0xbdc9d5));
 controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.enablePan=false;controls.minDistance=.7;controls.maxDistance=6;
 const resize=()=>{const w=mount.clientWidth||1,h=mount.clientHeight||1;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();};const observer=new ResizeObserver(resize);observer.observe(mount);resize();
 const draw=()=>{animation=requestAnimationFrame(draw);controls.update();renderer.render(scene,camera);};draw();
 for(const button of document.querySelectorAll('[data-motion-view]'))button.onclick=()=>{view=button.dataset.motionView;for(const b of document.querySelectorAll('[data-motion-view]')){const active=b===button;b.classList.toggle('active',active);b.setAttribute('aria-checked',String(active));}load();};
 $('motionResetPose').onclick=frame;load();
 window.addEventListener('beforeunload',()=>{++token;cancelAnimationFrame(animation);observer.disconnect();controls.dispose();disposeModel(model);renderer.dispose();},{once:true});
}catch(error){
 renderer?.dispose();renderer=null;window.__sceneError=String(error);status.textContent='Mô hình 3D không khả dụng · vẫn có thể đo và xuất dữ liệu';
}
