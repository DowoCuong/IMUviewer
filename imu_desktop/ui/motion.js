'use strict';
const $=id=>document.getElementById(id);
const labels={shoulder_plane:'Mặt phẳng nâng vai · YXY',shoulder_elevation:'Góc nâng vai',shoulder_axial:'Xoay dọc vai · YXY',elbow_flexion:'Gấp khuỷu',elbow_supination:'Ngửa cẳng tay',wrist_flexion:'Gấp cổ tay',wrist_radial:'Lệch quay cổ tay'};
const maneuvers={shoulder_flexion:['Nâng tay ra trước','Giữ thân ổn định. Bắt đầu nâng tay ra trước.'],shoulder_abduction:['Nâng tay sang bên','Giữ thân ổn định. Bắt đầu nâng tay sang bên đang đo.'],elbow_flexion:['Gấp / duỗi khuỷu','Giữ cánh tay ổn định. Bắt đầu gấp khuỷu.'],elbow_supination:['Sấp / ngửa cẳng tay','Khuỷu gần duỗi như tư thế chuẩn. Sau lấy mốc, sấp nhẹ rồi bắt đầu ghi theo chiều ngửa; không xoay vai.'],wrist_flexion:['Gấp / duỗi cổ tay','Giữ cẳng tay ổn định. Bắt đầu gấp lòng bàn tay.'],wrist_radial:['Lệch quay / trụ cổ tay','Giữ cẳng tay ổn định. Bắt đầu lệch về phía ngón cái.']};
const reasons={valid:'Hợp lệ',near_singularity:'Gần kỳ dị · không tách được',calibration_required:'Chưa đủ hiệu chuẩn / kiểm tra',missing_pair:'Thiếu cặp mẫu',stale:'Đã hết hạn mẫu'};
let desktop=null, collecting=null, rows=[], last=new Map(), count=0, connected=false;
let configuredModel='upper_limb_v1',configuredSide='right';
document.querySelectorAll('header a').forEach(link=>link.addEventListener('click',event=>{
 if(connected){event.preventDefault();message('Dừng kết nối trước khi chuyển màn hình.');}
}));
function call(action,values={}){return new Promise(resolve=>desktop.motion_action(action,JSON.stringify(values),resolve));}
function message(text){$('message').textContent=text||'';}
function setConnected(value){connected=value;for(const id of ['model','connect','replay'])$(id).disabled=value;$('side').disabled=value||$('model').value!=='upper_limb_v1';$('stop').disabled=!value;$('motionBadge').classList.toggle('online',value);$('motionBadge').classList.toggle('offline',!value);$('motionConnection').textContent=value?'Đã kết nối':'Chưa kết nối';$('motionState').textContent=value?'Đang nhận':'Sẵn sàng';}
function setup(){
 const upper=$('model').value==='upper_limb_v1';
 if(!upper)$('side').value='right';$('side').disabled=!upper||connected;
 $('motionHeading').textContent=upper?'Đo góc chi trên':'Đo góc chi dưới';
 $('focusLabel').textContent=upper?'Thân trên':'Thân dưới';
 for(const [id,text]of [['legendShoulder',upper?'Vai':'Hông'],['legendElbow',upper?'Khuỷu':'Gối'],['legendWrist',upper?'Cổ tay':'Cổ chân']])$(id).textContent=text;
 $('mapping').textContent=upper?'1 thân ngực · 2 cánh tay · 3 cẳng tay · 4 bàn tay':'1 chậu · 2 đùi · 3 cẳng chân · 4 bàn chân';
 $('pose').textContent=upper?'Tư thế chuẩn: thân thẳng, hai tay buông, khuỷu duỗi tự nhiên, lòng bàn tay hướng trước. Tư thế này tạo mốc vận hành; không tự đo được zero giải phẫu cá nhân.':'Tư thế đứng thẳng chuẩn như phiên gait hiện tại.';
 $('maneuver').replaceChildren();$('curve').replaceChildren();$('cards').replaceChildren();
 const ms=upper?Object.entries(maneuvers):[['hip',['Hông','Bắt đầu gấp hông.']],['knee',['Gối','Bắt đầu gấp gối.']],['ankle',['Cổ chân','Bắt đầu gấp mu chân.']]];
 for(const [key,[label]] of ms)$('maneuver').add(new Option(label,key));
 const angles=upper?Object.entries(labels):['hip','knee','ankle'].flatMap((j,i)=>['flexion','abduction','rotation'].map((c,k)=>[`${j}_${c}`,`${['Hông','Gối','Cổ chân'][i]} · ${['gấp/duỗi','dạng/khép','xoay'][k]}`]));
 const names=upper?{shoulder:'Vai',elbow:'Khuỷu',wrist:'Cổ tay'}:{hip:'Hông',knee:'Gối',ankle:'Cổ chân'};
 const primary=upper?{shoulder:'shoulder_elevation',elbow:'elbow_flexion',wrist:'wrist_flexion'}:{hip:'hip_flexion',knee:'knee_flexion',ankle:'ankle_flexion'};
 const groups=new Map();
 for(const [joint,name]of Object.entries(names)){const card=document.createElement('article');const color={shoulder:'hip',elbow:'knee',wrist:'ankle',hip:'hip',knee:'knee',ankle:'ankle'}[joint];card.className=`angle-card joint-card ${color}-card`;const title=document.createElement('h3');title.textContent=name;const secondary=document.createElement('div');secondary.className='secondary-readings';card.append(title,secondary);$('cards').append(card);groups.set(joint,{card,secondary});}
 const ordered=[...angles].sort(([a],[b])=>Number(b===primary[b.split('_')[0]])-Number(a===primary[a.split('_')[0]]));
 for(const [key,label] of ordered){const joint=key.split('_')[0],group=groups.get(joint),reading=document.createElement('div');reading.className='reading invalid';reading.dataset.key=key;const name=document.createElement('span');name.textContent=label;const value=document.createElement('strong');value.textContent='—';const quality=document.createElement('small');quality.textContent='Chưa có dữ liệu';reading.append(name,value,quality);if(key===primary[joint])group.card.insertBefore(reading,group.secondary);else group.secondary.append(reading);$('curve').add(new Option(label,key));}
 rows=[];last.clear();count=0;updateInstruction();render();
}
function updateInstruction(){$('instruction').textContent=maneuvers[$('maneuver').value]?.[1]||'Bắt đầu gấp theo chiều dương, rồi gấp/duỗi.';}
function render(){
 for(const card of $('cards').querySelectorAll('.reading')){const key=card.dataset.key;const joint=key.split('_')[0], entry=last.get(joint);const valid=entry&&Number.isFinite(entry.angles[key]);card.querySelector('strong').textContent=valid?entry.angles[key].toFixed(2)+'°':'—';const quality=entry?(reasons[entry.quality[key]]||entry.quality[key]||'Thiếu dữ liệu'):'Chưa có mẫu mới';card.querySelector('small').textContent=quality;card.title=quality;card.classList.toggle('invalid',!valid||!entry?.measured);}
 const ready=last.size===3&&[...last.values()].every(r=>r.measured);
 $('state').textContent=collecting?'Đang ghi hiệu chuẩn':ready?'Đã hiệu chuẩn và kiểm tra':'Chờ mẫu đo và hiệu chuẩn hợp lệ';$('state').classList.toggle('ready',ready);
 $('motionCount').textContent=count.toLocaleString();
 $('motionDuration').textContent=((rows.at(-1)?.t_ms||0)/1000).toFixed(1)+' s';
 window.motionView?.update(last,configuredModel,configuredSide);
 draw();
}
function draw(){const canvas=$('chart'),ctx=canvas.getContext('2d'),key=$('curve').value,joint=key?.split('_')[0];const w=canvas.clientWidth||1,h=canvas.clientHeight||1,dpi=Math.min(devicePixelRatio||1,2);if(canvas.width!==Math.round(w*dpi)||canvas.height!==Math.round(h*dpi)){canvas.width=Math.round(w*dpi);canvas.height=Math.round(h*dpi);}ctx.setTransform(dpi,0,0,dpi,0,0);ctx.clearRect(0,0,w,h);const points=rows.filter(r=>r.joint===joint);const valid=points.filter(r=>Number.isFinite(r.angles[key]));$('motionChartEmpty').classList.toggle('hidden',valid.length>0);if(!valid.length)return;const end=points.at(-1).t_ms,start=end-30000;const values=valid.filter(r=>r.t_ms>=start).map(r=>r.angles[key]);const lo=Math.min(-10,...values),hi=Math.max(10,...values);ctx.font='10px Segoe UI';ctx.fillStyle='#82909e';ctx.fillText(`${hi.toFixed(1)}°`,4,14);ctx.fillText(`${lo.toFixed(1)}°`,4,h-12);ctx.fillText('30 giây gần nhất',Math.max(50,w-120),h-5);ctx.strokeStyle='#e7ecf1';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(44,h-25);ctx.lineTo(w-10,h-25);ctx.stroke();ctx.strokeStyle='#1769e0';ctx.lineWidth=2;ctx.beginPath();let prev=null;for(const r of points){if(r.t_ms<start)continue;const v=r.angles[key];if(!Number.isFinite(v)){prev=null;continue;}const x=45+(r.t_ms-start)/30000*Math.max(1,w-60),y=h-25-(v-lo)/(hi-lo)*Math.max(1,h-45);if(!prev||r.t_ms-prev.t_ms>30||r.calibration_id!==prev.calibration_id)ctx.moveTo(x,y);else ctx.lineTo(x,y);prev=r;}ctx.stroke();}
window.handleMotionBatch=batch=>{for(const p of batch){if(p.kind==='sample'){const r=p.row;last.set(r.joint,r);rows.push(r);count=p.count;}else if(p.kind==='invalid'){last.delete(p.joint);}else if(p.kind==='status'){message(p.message);}else if(p.kind==='health'){
 const radio=p.health.radio?.nodes||[];
 const rates=radio.filter(n=>n.receiving&&Number.isFinite(n.received_hz)).map(n=>n.received_hz);$('motionRate').textContent=rates.length?Math.min(...rates).toFixed(1)+' Hz/node':'— Hz/node';
 $('health').textContent=radio.map(n=>{const c=p.health.calibration?.[String(n.node)];const cal=Number.isInteger(c)?[6,4,2,0].map(s=>(c>>s)&3).join('/'):'—';const h=p.health.quality_nodes?.[String(n.node)];const health=h?(['chất lượng chưa biết','fusion tốt','fusion lỗi'][h.health]+' · epoch '+h.sensor_epoch+' · tuổi cal/health '+h.cal_age_ms+'/'+h.health_age_ms+' ms'):'chưa có health';return `Node ${n.node}: ${health} · ${n.receiving?'đang nhận':'chưa nhận'} · ${n.received_hz?.toFixed(1)||'—'} gói/s · gap ${n.sequence_gaps||0} · SYS/GYR/ACC/MAG ${cal}`;}).join('\n')+'\nGói/gap từ hub USB, không phải RF loss thuần. Mức calibration là giá trị gần nhất.';
 for(const [j,r] of last){const pair={shoulder:[1,2],elbow:[2,3],wrist:[3,4],hip:[1,2],knee:[2,3],ankle:[3,4]}[j];if(pair.some(n=>!p.health.nodes?.[String(n)]))last.delete(j);}
 }}if(rows.length>12000)rows=rows.slice(-12000);render();};
async function refresh(){const ports=await new Promise(resolve=>desktop.list_ports(resolve));$('port').replaceChildren(new Option('Chọn cổng hub',''));for(const p of ports)$('port').add(new Option(`${p.device} · ${p.description}`,p.device));}
async function action(name,values){try{const result=await call(name,values);if(!result.ok){if(!result.cancelled)message(result.message);return result;}message(result.message||'Đã thực hiện');return result;}catch(e){message(String(e));return {ok:false};}}
for(const id of ['model','side'])$(id).onchange=async()=>{if(count&&!window.confirm('Bắt đầu phiên mới? Hãy xuất phiên hiện tại trước nếu cần giữ.')){$('model').value=configuredModel;$('side').value=configuredSide;return;}if($('model').value==='lower_limb_v1')$('side').value='right';const r=await action('configure',{model_id:$('model').value,side:$('side').value});if(r.ok){configuredModel=$('model').value;configuredSide=$('side').value;setup();}else{$('model').value=configuredModel;$('side').value=configuredSide;}};
$('refresh').onclick=refresh;
$('connect').onclick=async()=>{const r=await action('connect',{port:$('port').value});if(r.ok)setConnected(true);};
$('stop').onclick=async()=>{const r=await action('stop');if(r.ok){setConnected(false);last.clear();render();}};
$('neutral').onclick=async()=>{const r=await action('neutral');if(r.ok){last.clear();render();message('Đã lấy mốc. Ghi trục và kiểm tra độc lập từng chuyển động.');}};
$('maneuver').onchange=updateInstruction;
for(const [id,validate] of [['fit',false],['validate',true]])$(id).onclick=async()=>{const name=$('maneuver').value;if(collecting&&(collecting.name!==name||collecting.validate!==validate)){message('Hoàn tất lần ghi đang chạy trước');return;}const finish=Boolean(collecting);const r=await action('functional',{name,finish,validate});if(r.ok){collecting=finish?null:{name,validate};$(id).textContent=finish?(validate?'3. Ghi kiểm tra độc lập':'2. Ghi trục'):'Hoàn tất lần ghi';message(JSON.stringify(r.report,null,2));}else if(finish){collecting=null;$(id).textContent=validate?'3. Ghi kiểm tra độc lập':'2. Ghi trục';}};
for(const id of ['load_profile','save_profile','export','replay'])$(id).onclick=async()=>{const r=await action(id);if(r.ok&&id==='replay'){configuredModel=r.state.model_id;configuredSide=r.state.side;$('model').value=configuredModel;$('side').value=configuredSide;setup();rows=r.tail||[];count=r.state.rows;for(const [j,row] of Object.entries(r.latest||{}))last.set(j,row);render();message('Replay hoàn tất. Đồ thị hiển thị phần cuối có giới hạn; có thể xuất toàn bộ kết quả. Đây không phải lần đo phần cứng mới.');}};
$('curve').onchange=draw;
$('clearMotionChart').onclick=()=>{rows=[];draw();};
new ResizeObserver(draw).observe($('chart').parentElement);
new QWebChannel(qt.webChannelTransport,async channel=>{desktop=channel.objects.desktop;const r=await action('state');if(r.ok){configuredModel=r.state.model_id;configuredSide=r.state.side;$('model').value=configuredModel;$('side').value=configuredSide;setup();count=r.state.rows;setConnected(r.connected);render();}await refresh();});
