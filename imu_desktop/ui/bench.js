'use strict';
const $ = id => document.getElementById(id);
let desktop = null, state = {}, samples = [], toastTimer = null, paintPending = false;
const options = () => ({imu_port:$('benchImuPort').value, encoder_port:$('benchEncoderPort').value,
  motor_port:$('benchMotorPort').value, protocol:$('benchProtocol').value,
  parent:Number($('benchParent').value), child:Number($('benchChild').value), channel:Number($('benchChannel').value),
  axis:$('benchAxis').value, sign:Number($('benchSign').value), counts_per_rev:Number($('benchCpr').value),
  max_gap_ms:Number($('benchGap').value)});
function toast(message, error=false) {
  if (!message) return;
  $('toast').textContent=message; $('toast').classList.toggle('error',error); $('toast').classList.add('visible');
  clearTimeout(toastTimer); toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),5000);
}
function call(method,...args) {
  return new Promise((resolve,reject)=>{try {desktop[method](...args,resolve);} catch(e) {reject(e);}});
}
async function action(name,args={}) {
  try {
    const result=await call('bench_action',name,JSON.stringify(args));
    if (!result.ok) toast(result.message,true);
    else if(result.state) updateState(result.state);
    if(result.ok && ['connect','clear'].includes(name)) { samples=[]; paint(); }
    return result;
  } catch(e) {toast(String(e),true);return {ok:false};}
}
function degrees(v) {return v == null || !Number.isFinite(Number(v)) ? '—' : Number(v).toFixed(3)+'°';}
function updateRadio(radio={}) {
  state.radio=radio;
  $('benchExport').disabled=state.recording || !((state.metrics?.count||0)>0 || (radio.nodes||[]).some(r=>r.packets>0));
  const body=$('benchRadioRows'); body.replaceChildren();
  for(const r of radio.nodes||[]) {
    const tr=document.createElement('tr');
    const kind=r.test_packets ? (r.timed_packets?'Lẫn đo/tạo tải':'Tạo tải') : (r.timed_packets?'IMU thật':'Chưa có mẫu đo');
    const cal=r.calibration_raw;
    const h=r.quality;const quality=h?.sensor_epoch!=null?' · '+(['chưa biết','fusion tốt','fusion lỗi'][h.health])+' · epoch '+h.sensor_epoch:'';
    const levels=cal==null?'—':[6,4,2,0].map(shift=>(cal>>shift)&3).join(' / ');
    for(const text of [`${r.node} · ${r.receiving?'đang nhận':'chưa nhận gần đây'}`,kind,r.packets||0,r.received_hz==null?'—':r.received_hz.toFixed(1),r.sequence_gaps||0,levels+quality]) {
      const td=document.createElement('td');td.textContent=String(text);tr.append(td);
    }
    if(cal!=null && cal!==255) tr.lastElementChild.style.color='#a13c3c';
    body.append(tr);
  }
  const hub=radio.hub_quality_pages?.['2'];
  if(hub) {
    const tr=document.createElement('tr'); const td=document.createElement('td'); td.colSpan=6;
    const c=hub.health_counters;
    td.textContent=`Hub RX (snapshot ${hub.health_time_local_us} ?s): invalid ${c[0]}, payload ${c[1]}, pipe/type ${c[2]}, queue ${c[3]}, FIFO ${c[4]}`;
    tr.append(td);body.append(tr);
  }
  const tests=(radio.nodes||[]).filter(r=>r.test_packets);
  $('benchRadioTiming').textContent=tests.map(r=>`Node ${r.node}: trễ bắt đầu lớn nhất ${r.max_tx_start_lateness_us} µs; nhận xong ngoài slot ${r.rx_end_outside_slot} gói`).join(' · ') || 'Đợi gói tạo tải để kiểm tra thời điểm TDMA.';
}
function updateNumbers(metrics={},latest=null) {
  $('benchRmse').textContent=degrees(metrics.rmse_deg); $('benchBias').textContent=degrees(metrics.bias_deg);
  $('benchMax').textContent='Lớn nhất '+degrees(metrics.max_abs_deg);
  $('benchSamples').textContent=Number(metrics.count||0).toLocaleString()+' mẫu đối chiếu';
  if(latest) {
    $('benchImu').textContent=degrees(latest.imu_deg); $('benchEncoder').textContent=degrees(latest.encoder);
    $('benchOffAxis').textContent='Góc quay lệch trục '+degrees(latest.off_axis_deg);
  } else {
    $('benchImu').textContent=$('benchEncoder').textContent='—';
    $('benchOffAxis').textContent='Góc quay lệch trục —';
  }
  $('benchExport').disabled=state.recording || !(metrics.count>0 || (state.radio?.nodes||[]).some(r=>r.packets>0));
}
function updateState(next) {
  state=next;
  document.querySelectorAll('.setup').forEach(el=>el.disabled=state.connected);
  $('benchBadge').textContent=state.recording?'Đang ghi':(state.connected?'Đã kết nối':'Chưa kết nối');
  $('benchBadge').classList.toggle('online',!!state.connected); $('benchBadge').classList.toggle('offline',!state.connected);
  $('benchConnect').disabled=state.connected; $('benchDisconnect').disabled=!state.connected;
  $('benchZero').disabled=!state.connected || !state.options?.encoder_port || state.recording || (state.metrics?.count||0)>0;
  $('benchStart').disabled=!state.connected || !state.options?.encoder_port || !state.calibrated || state.recording || (state.metrics?.count||0)>0;
  $('benchStop').disabled=!state.recording; $('benchClear').disabled=!!state.connected;
  $('motorMove').disabled=$('motorCycle').disabled=!state.connected || state.motor_state!=='idle';
  $('motorStop').disabled=$('motorStopTop').disabled=state.motor_state==='disconnected';
  const motorLabels={disconnected:'chưa kết nối',starting:'đang khởi động',idle:'đã dừng',moving:'đang quay',cycling:'đang quay lặp',stopping:'đang dừng mềm',unknown:'cần kiểm tra lại'};
  $('motorState').textContent='Uno '+(motorLabels[state.motor_state]||'chưa rõ trạng thái');
  if(state.connected && state.options){
    const fields={imu_port:'benchImuPort',encoder_port:'benchEncoderPort',motor_port:'benchMotorPort',protocol:'benchProtocol',parent:'benchParent',child:'benchChild',channel:'benchChannel',axis:'benchAxis',sign:'benchSign',counts_per_rev:'benchCpr',max_gap_ms:'benchGap'};
    for(const [key,id]of Object.entries(fields))if(state.options[key]!=null)$(id).value=String(state.options[key]);
  }
  $('benchStatus').textContent=state.message || '';
  const c=state.counts||{};
  $('benchCounters').textContent=`Mẫu IMU ${c.imu_frames||0} · encoder ${c.encoder_samples||0} · đối chiếu ${c.paired_samples||0} · mất mẫu chuẩn ${c.encoder_gaps||0} · lỗi thiết bị ${c.device_failures||0}`;
  if(state.connected && !state.options?.encoder_port) $('benchCounters').textContent='Chỉ kiểm tra đường truyền; chưa có encoder nên chưa tính góc đối chiếu và RMSE.';
  updateNumbers(state.metrics,state.latest);
  updateRadio(state.radio);
}
window.handleBenchBatch = batch => {
  for(const item of batch) {
    if(item.kind==='state') updateState(item);
    else if(item.kind==='radio') updateRadio(item.radio);
    else if(item.kind==='sample') {
      updateNumbers(item.metrics,item.row);
      state.metrics=item.metrics;
      samples.push(item.row);
      while(samples.length>1 && samples.at(-1).t_ms-samples[0].t_ms>30000) samples.shift();
      if(samples.length>5000) samples=samples.slice(-5000);
    }
  }
  if(!paintPending) {paintPending=true;requestAnimationFrame(()=>{paintPending=false;paint();});}
};
function draw(canvas, series) {
  const rect=canvas.getBoundingClientRect(), scale=window.devicePixelRatio||1;
  canvas.width=Math.max(1,Math.round(rect.width*scale)); canvas.height=Math.max(1,Math.round(rect.height*scale));
  const ctx=canvas.getContext('2d');ctx.scale(scale,scale);
  const w=rect.width,h=rect.height,p={left:52,right:12,top:12,bottom:25};
  const plotW=w-p.left-p.right,plotH=h-p.top-p.bottom;
  if(plotW<=0||plotH<=0)return;
  ctx.font='10px Segoe UI';ctx.fillStyle='#81909e';ctx.strokeStyle='#e3e9f0';ctx.lineWidth=1;
  if(!samples.length){ctx.fillText('Bắt đầu phép thử để ghi và vẽ dữ liệu đối chiếu.',p.left,Math.max(20,h/2));return;}
  const first=samples[0].t_ms,last=Math.max(first+1,samples.at(-1).t_ms);
  let low=Infinity,high=-Infinity;
  for(const row of samples)for(const [key]of series){low=Math.min(low,row[key]);high=Math.max(high,row[key]);}
  const pad=Math.max(.2,(high-low)*.12);low-=pad;high+=pad;
  const x=t=>p.left+(t-first)/(last-first)*plotW, y=v=>p.top+(high-v)/(high-low)*plotH;
  const ticks=Math.max(1,Math.min(4,Math.floor(plotH/26)));
  for(let i=0;i<=ticks;i++){
    const yy=p.top+i*plotH/ticks;ctx.beginPath();ctx.moveTo(p.left,yy);ctx.lineTo(w-p.right,yy);ctx.stroke();
    ctx.fillText((high-i*(high-low)/ticks).toFixed(1),2,yy+3);
    const time=first+i*(last-first)/ticks;ctx.fillText((time/1000).toFixed(1)+'s',x(time)-12,h-5);
  }
  for(const [key,color] of series){ctx.strokeStyle=color;ctx.lineWidth=1.3;ctx.beginPath();let previous=null;
    for(const row of samples){if(!previous||row.t_ms-previous.t_ms>30)ctx.moveTo(x(row.t_ms),y(row[key]));else ctx.lineTo(x(row.t_ms),y(row[key]));previous=row;}
    ctx.stroke();
  }
}
function paint(){draw($('benchAngles'),[['imu_deg','#2374df'],['encoder','#dd8c26']]);draw($('benchErrors'),[['error_deg','#c44f56']]);}
async function refresh() {
  try {
    const ports=await call('list_ports');
    for(const id of ['benchImuPort','benchEncoderPort','benchMotorPort']) {
      const select=$(id),prev=select.value;select.replaceChildren();
      const empty=document.createElement('option');empty.value='';empty.textContent=id==='benchMotorPort'?'Không dùng điều khiển motor':'Chọn cổng kết nối';select.append(empty);
      for(const item of ports){const opt=document.createElement('option');opt.value=item.device;opt.textContent=`${item.device} — ${item.description}`;select.append(opt);}
      if(ports.some(p=>p.device===prev))select.value=prev;
    }
  }catch(e){toast(String(e),true);}
}
for(const [id,name] of [['benchConnect','connect'],['benchDisconnect','disconnect'],['benchZero','zero'],['benchStart','start'],['benchStop','stop'],['benchClear','clear']]) {
  $(id).addEventListener('click',async()=>{const button=$(id);button.disabled=true;await action(name,name==='connect'?options():{});button.disabled=false;if(state)updateState(state);});
}
for(const [id,command] of [['motorMove','M'],['motorCycle','C'],['motorStop','S'],['motorStopTop','S']]) {
  $(id).addEventListener('click',()=>action('motor',{command,angle1:Number($('motorAngle1').value),angle2:Number($('motorAngle2').value)}));
}
$('benchExport').addEventListener('click',async()=>{try{const result=await call('bench_export');toast(result.message,!result.ok&&!result.cancelled);}catch(e){toast(String(e),true);}});
$('benchRefresh').addEventListener('click',refresh);
for(const id of ['gaitLink','motionLink']) $(id).addEventListener('click',event=>{if(state.connected){event.preventDefault();toast('Ngắt kết nối phép thử trước khi chuyển màn hình.',true);}});
new ResizeObserver(paint).observe($('benchAngles').parentElement);
new ResizeObserver(paint).observe($('benchErrors').parentElement);
new QWebChannel(qt.webChannelTransport,async channel=>{desktop=channel.objects.desktop;await refresh();await action('state');});
