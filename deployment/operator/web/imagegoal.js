'use strict';
// Task controls use only the ImageGoal runtime API. There is no velocity or SDK API here.
const $ = id => document.getElementById(id);
let goalBase64 = null, goalSelectionId = null, frozenImageSelectionId = null, draftGoal = false, lastStatus = null, lastSequence = 0;
let busy = false, eventRows = [], currentRgbURL = null, currentGoalURL = null, seenGoal = null;
let stopGeneration = 0, cancelInFlight = false;
const imageVersions={rgb:0,goal:0};
const finite = x => typeof x === 'number' && Number.isFinite(x);
const fmt = (x, unit='', digits=1) => finite(x) ? `${x.toFixed(digits)}${unit}` : '—';
const ageMs = sensor => finite(sensor.age_ms) ? sensor.age_ms : finite(sensor.age_s) ? sensor.age_s*1000 : null;
function text(id, value) { $(id).textContent = value ?? '—'; }
function metric(parent, label, value) {
  const d=document.createElement('div'); d.className='metric'; d.textContent=label;
  const b=document.createElement('strong'); b.textContent=value; d.append(b); parent.append(d);
}
async function api(path, body) {
  const response=await fetch(`/api${path}`, {method:body===undefined?'GET':'POST',
    headers:body===undefined?{}:{'Content-Type':'application/json'},
    body:body===undefined?undefined:JSON.stringify(body), signal:AbortSignal.timeout(8000)});
  let result; try {result=await response.json();} catch {throw new Error(`HTTP ${response.status}`);}
  if(!response.ok) throw new Error(result.error || result.message || JSON.stringify(result));
  return result;
}
async function action(path, body={}) {
  if(busy) return; busy=true;updateButtons();text('action-status','正在提交操作…');const generation=stopGeneration;
  try {
    if(path==='/tasks'||path==='/task/goal'||path==='/native/start'){
      const current=await api('/status');
      if(generation!==stopGeneration)return;
      body={...body,expected_epoch:current.epoch,expected_session_id:current.session_id};
    }
    await api(path,body);
    if(generation!==stopGeneration) await api('/task/cancel',{reason:'operator_stop_during_pending_request'});
    text('action-status',generation!==stopGeneration?'任务已停止。':'操作已提交，等待运行状态更新。'); await pollStatus();}
  catch(e) {text('action-status',e.message);} finally {busy=false;updateButtons();}
}
async function showBlob(id, blob) {
  const version=++imageVersions[id],u=URL.createObjectURL(blob), img=$(id);
  try {const preload=new Image();preload.src=u;await preload.decode();}
  catch(e){URL.revokeObjectURL(u);throw e;}
  if(version!==imageVersions[id]){URL.revokeObjectURL(u);return;}
  img.src=u;
  img.parentElement.classList.add('has-image');
  if(id==='rgb') {if(currentRgbURL) URL.revokeObjectURL(currentRgbURL);currentRgbURL=u;}
  else {if(currentGoalURL) URL.revokeObjectURL(currentGoalURL);currentGoalURL=u;}
}
async function sealDraft(blob) {
  if(blob.size>12*1024*1024) throw new Error('目标图超过 12 MB。');
  const bitmap=await createImageBitmap(blob);
  if(bitmap.width>4096 || bitmap.height>2160) {bitmap.close();throw new Error('目标图最大为 4096×2160。');}
  const c=document.createElement('canvas');c.width=bitmap.width;c.height=bitmap.height;
  c.getContext('2d').drawImage(bitmap,0,0);bitmap.close();
  goalBase64=c.toDataURL('image/jpeg',0.95).split(',')[1];draftGoal=true;
  await showBlob('goal',await (await fetch(`data:image/jpeg;base64,${goalBase64}`)).blob());
  text('goal-version','待提交目标'); text('action-status','目标图已就绪。开始新任务，或替换当前任务目标。');
  updateButtons();
}
async function freezeGoal(path,body={}) {
  if(busy)return;busy=true;updateButtons();const generation=stopGeneration;
  try {
    const state=await api('/status');if(generation!==stopGeneration)return;
    const frozen=await api(path,{...body,expected_epoch:state.epoch,expected_session_id:state.session_id});
    if(generation!==stopGeneration)return;
    const selection=frozen.selection??frozen.status?.goal_selection;
    if(selection?.ready!==true||typeof selection.selection_id!=='string')throw new Error('运行时尚未确认冻结目标图。');
    goalSelectionId=selection.selection_id;lastStatus=frozen.status??lastStatus;draftGoal=false;
    goalBase64=frozen.goal?.rgb_jpeg_base64??goalBase64;
    if(goalBase64){await showBlob('goal',await(await fetch(`data:image/jpeg;base64,${goalBase64}`)).blob());frozenImageSelectionId=selection.selection_id;}
    text('goal-version','目标图已冻结');text('action-status','目标图已冻结，可以开始任务。');
  }catch(e){goalSelectionId=null;text('action-status',e.message);}finally{busy=false;updateButtons();}
}
function updateButtons() {
  const active=!!lastStatus?.task && !['IDLE','CANCELLED','SUCCEEDED','FAILED','COMPLETED','TIMED_OUT'].includes(String(lastStatus?.state).toUpperCase());
  const lightnav=$('backend').value==='lightnav',cap=lastStatus?.execution_capability||{},mode=$('navigation-mode').value,native=$('execution-mode').value==='supervised_native';
  const imagegoal=lightnav&&mode==='imagegoal';
  const modeSupported=Array.isArray(cap.task_modes)&&cap.task_modes.includes(mode)&&(!native||(Array.isArray(cap.supervised_native_task_modes)&&cap.supervised_native_task_modes.includes(mode)))&&(!imagegoal||cap.lightnav_imagegoal_adapter===true);
  const nativeSupported=!native||(Array.isArray(cap.supervised_native_backends)?cap.supervised_native_backends.includes($('backend').value):!lightnav);
  $('start').disabled=busy || !lastStatus || !nativeSupported || (lightnav?(!modeSupported||(imagegoal?(!goalSelectionId||frozenImageSelectionId!==goalSelectionId):!$('instruction').value.trim())):!goalBase64); $('replace-goal').disabled=busy || lightnav || !goalBase64 || !active;
  $('pause').disabled=busy || !active; $('resume').disabled=busy || !active; $('complete').disabled=busy || !active;
}
$('goal-file').addEventListener('change',async e=>{try {if(e.target.files[0]) {goalSelectionId=null;await sealDraft(e.target.files[0]);await freezeGoal('/goal/import',{goal:{rgb_jpeg_base64:goalBase64}});}} catch(err) {text('action-status',err.message);}});
$('capture').onclick=()=>freezeGoal('/goal/capture');
function updateExecutionMode() {
  const native=$('execution-mode').value==='supervised_native';
  const lightnav=$('backend').value==='lightnav',imagegoal=lightnav&&$('navigation-mode').value==='imagegoal',language=lightnav&&!imagegoal;
  $('instruction-label').hidden=$('navigation-mode-label').hidden=!lightnav;
  $('goal-preview').hidden=$('goal-controls').hidden=language;$('imagegoal-adaptation-note').hidden=!imagegoal;
  text('instruction-title',imagegoal?'补充指令（可选）':'LightNav 导航指令（必填）');
  $('instruction').placeholder=imagegoal?'可留空，使用冻结目标图导航':'Go to the coat rack beside the windows.';
  text('goal-version',language?'官方 RGB 历史 + 文字指令':goalSelectionId?'目标图已冻结':'采集或导入完整目标图');
  text('task-heading',language?'LightNav 官方导航':imagegoal?'LightNav ImageGoal':'目标图与任务');
  $('timeout').max=native?'120':'3600';
  if(native&&Number($('timeout').value)>120)$('timeout').value='60';
  $('start').textContent=native?'开始监督真机任务':$('execution-mode').value==='operator_run'?'开始真机执行':'开始影子任务';
  updateButtons();
}
$('backend').onchange=()=>{updateExecutionMode();text('action-status',$('backend').value==='lightnav'?'选择 ObjNav 或 VLN，填写文字指令后开始；无需目标图。':'选择完整目标图后开始。');};
$('execution-mode').onchange=updateExecutionMode;
$('navigation-mode').onchange=updateExecutionMode;
$('instruction').oninput=updateButtons;
$('start').onclick=()=>{
  const timeout=Number($('timeout').value);
  const maxTimeout=$('execution-mode').value==='supervised_native'?120:3600;
  if(!finite(timeout)||timeout<5||timeout>maxTimeout) return text('action-status',`时限需为 5–${maxTimeout} 秒。`);
  const native=$('execution-mode').value==='supervised_native';
  const lightnav=$('backend').value==='lightnav',imagegoal=lightnav&&$('navigation-mode').value==='imagegoal';
  if(lightnav&&!imagegoal&&!$('instruction').value.trim())return text('action-status','请填写 LightNav 导航指令。');
  if($('start').disabled)return;
  draftGoal=false; action(native?'/native/start':'/tasks',{...(lightnav?{task_mode:$('navigation-mode').value,...(imagegoal?{selection_id:goalSelectionId}:{})}:{goal:{rgb_jpeg_base64:goalBase64}}),backend:$('backend').value,
    execution_mode:$('execution-mode').value,timeout_s:native?60:timeout,instruction:lightnav?$('instruction').value.trim():'',
    ...(native?{operator_present:true}:{})});
};
$('replace-goal').onclick=()=>{draftGoal=false;action('/task/goal',{goal:{rgb_jpeg_base64:goalBase64}});};
$('pause').onclick=()=>action('/task/pause'); $('resume').onclick=()=>action('/task/resume');
$('cancel').onclick=async()=>{
  stopGeneration++;
  if(cancelInFlight) return;
  cancelInFlight=true;
  try {await api(lastStatus?.task?.execution_mode==='supervised_native'?'/native/stop':'/task/cancel',{reason:'operator_stop'});text('action-status','停止已提交。');await pollStatus();}
  catch(e){text('action-status',e.message);}finally{cancelInFlight=false;}
};
$('complete').onclick=()=>action('/task/complete',{evidence:'operator_confirmed'});
function drawPath(s) {
  const canvas=$('path'),ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height;
  ctx.clearRect(0,0,w,h); const p=s.plan||{};
  const previewOnly=!p.trajectory_odom_m_rad?.length && !!s.preview?.trajectory_body_m?.length;
  const lines=[[previewOnly?s.preview.trajectory_body_m:p.trajectory_odom_m_rad,'#23c483'],[p.reference_odom_m_rad,'#f4cc45'],[p.prediction_odom_m_rad,'#51a7fa']];
  const all=lines.flatMap(([a])=>Array.isArray(a)?a:[]).filter(a=>Array.isArray(a)&&finite(a[0])&&finite(a[1]));
  const pose=previewOnly?null:s.sensors?.odom?.pose_m_rad;
  if(Array.isArray(pose)) all.push(pose);
  if(!all.length) {ctx.fillStyle='#91aaa2';ctx.font='16px sans-serif';ctx.fillText('等待有效路径和反馈',235,195);return;}
  const xmin=Math.min(...all.map(a=>a[0]))-.15,xmax=Math.max(...all.map(a=>a[0]))+.15;
  const ymin=Math.min(...all.map(a=>a[1]))-.15,ymax=Math.max(...all.map(a=>a[1]))+.15;
  const scale=Math.min((w-80)/Math.max(.6,xmax-xmin),(h-70)/Math.max(.6,ymax-ymin));
  const cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;
  const xy=a=>[w/2+(a[1]-cy)*-scale,h/2-(a[0]-cx)*scale];
  ctx.strokeStyle='#19372b';ctx.lineWidth=1;
  for(let x=Math.floor(xmin/.2)*.2;x<=xmax;x+=.2) {const a=xy([x,ymin]),b=xy([x,ymax]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
  for(let y=Math.floor(ymin/.2)*.2;y<=ymax;y+=.2) {const a=xy([xmin,y]),b=xy([xmax,y]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
  for(const [a,color] of lines) {if(!Array.isArray(a)||!a.length)continue;ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();a.forEach((v,i)=>{const q=xy(v);i?ctx.lineTo(...q):ctx.moveTo(...q);});ctx.stroke();}
  if(Array.isArray(pose)) {const a=xy(pose),b=xy([pose[0]+.08*Math.cos(pose[2]),pose[1]+.08*Math.sin(pose[2])]);ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(...a,5,0,2*Math.PI);ctx.fill();ctx.strokeStyle='#fff';ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
  ctx.fillStyle='#91aaa2';ctx.font='12px monospace';ctx.fillText('网格 0.2 m · ↑ x · ← y',18,h-14);
}
async function pollStatus() {
  try {
    const s=await api('/status');
    if(lastStatus && s.session_id!==lastStatus.session_id){lastSequence=0;eventRows=[];seenGoal=null;goalSelectionId=null;frozenImageSelectionId=null;}
    if(s.goal_selection?.ready===true&&s.goal_selection.mode==='frozen')goalSelectionId=s.goal_selection.selection_id;else goalSelectionId=null;
    lastStatus=s;text('connection','已连接');$('connection').className='pill online';
    text('profile',s.profile||s.mode||'SHADOW');text('task-state',s.state||'IDLE');
    const synthetic=s.synthetic===true || String(s.profile||s.mode).includes('sim');
    const native=s.task?.execution_mode==='supervised_native';
    const physical=s.task?.execution_mode==='operator_run'||native;
    text('mode-notice',synthetic?'模拟反馈 · 真实模型与 MPC · 不代表真实机器人导航':native?
      '监督真机测试 · 宇树原生避障 · 手动停止':physical?
      '真机执行任务 · 是否允许输出由当前反馈、标定、计划与执行出口共同检查':
      '影子运行 · 模型与 MPC 数据仅供观测');
    $('mode-notice').className=physical?'mode-banner error':'mode-banner';
    const cap=s.execution_capability||{};
    const canExecute=cap.operator_run_configured===true&&cap.calibration_verified===true&&Array.isArray(cap.blocking_reasons)&&cap.blocking_reasons.length===0;
    const canNative=cap.supervised_native_configured===true&&cap.operator_supervised===true;
    $('operator-run-option').disabled=!canExecute;
    $('supervised-native-option').disabled=!canNative;
    if(($('execution-mode').value==='operator_run'&&!canExecute)||($('execution-mode').value==='supervised_native'&&!canNative))$('execution-mode').value='shadow';
    updateExecutionMode();
    const sensors=s.sensors||{},rgb=sensors.rgb||{},odom=sensors.odom||{},depth=sensors.depth||{};
    text('camera-age',fmt(ageMs(rgb),' ms'));
    text('sensor-note',rgb.timestamp_source||s.timestamp_source||'等待源时间信息');
    $('sensor-metrics').replaceChildren();
    metric($('sensor-metrics'),'RGB 年龄',fmt(ageMs(rgb),' ms'));metric($('sensor-metrics'),'ODOM 年龄',fmt(ageMs(odom),' ms'));
    metric($('sensor-metrics'),'深度年龄',fmt(ageMs(depth),' ms'));metric($('sensor-metrics'),'前方净空',fmt(depth.clearance_m,' m',2));
    const v=native?(s.native_execution?.authorized===true?s.native_execution.command||{}:{vx_m_s:0,wz_rad_s:0}):s.proposal||{},m=s.metrics||{};$('control-metrics').replaceChildren();
    metric($('control-metrics'),native?'vx · 实际候选命令':physical?'vx · 候选命令':'vx · 影子',fmt(v.vx_m_s,' m/s',3));metric($('control-metrics'),native?'wz · 实际候选命令':physical?'wz · 候选命令':'wz · 影子',fmt(v.wz_rad_s,' rad/s',3));
    metric($('control-metrics'),'模型往返',fmt(s.preview?.round_trip_ms??s.plan?.round_trip_ms??m.last_inference_ms??m.last_rtt_ms,' ms'));
    metric($('control-metrics'),'接收 / 丢弃',`${m.inference_accepted??'—'} / ${m.inference_rejected??'—'}`);
    metric($('control-metrics'),'模型时间距离',fmt(s.preview?.distance_steps,' steps',2));
    metric($('control-metrics'),'剩余时限',fmt(s.task?.remaining_s,' s',0));
    text('path-frame',s.plan?.odom_frame_id||(s.preview?'模型局部坐标 · 待 odom 锚定':'无有效参考系'));
    const gates=Array.isArray(s.gates)?s.gates:Object.entries(s.gates||{}).filter(([,v])=>v!==true).map(([k,v])=>`${k}: ${v}`);
    $('gates').replaceChildren();for(const g of gates.length?gates:['当前检查无阻断']) {const li=document.createElement('li');li.textContent=typeof g==='string'?g:JSON.stringify(g);$('gates').append(li);}
    $('identity').replaceChildren();for(const [k,v] of Object.entries({SESSION:s.session_id,TASK:s.task?.task_id||s.task?.id,BACKEND:s.task?.backend,GOAL:s.task?.goal_id,VERSION:s.task?.goal_version})) {const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=v??'—';$('identity').append(dt,dd);}
    if(!draftGoal&&$('backend').value!=='lightnav') text('goal-version',s.task?`目标版本 ${s.task.goal_version??'—'}`:'尚未设置');
    if($('backend').value==='lightnav'&&$('navigation-mode').value==='imagegoal'&&goalSelectionId&&frozenImageSelectionId!==goalSelectionId){const selectionId=goalSelectionId,r=await fetch('/api/goal.jpg?selection_id='+encodeURIComponent(selectionId),{cache:'no-store'});if(r.ok&&goalSelectionId===selectionId){await showBlob('goal',await r.blob());frozenImageSelectionId=selectionId;}}
    const key=s.task?`${s.task.task_id||s.task.id}/${s.task.goal_version}`:null;
    if(!draftGoal&&s.task?.backend==='nomad'&&key&&key!==seenGoal) {const r=await fetch('/api/goal.jpg',{cache:'no-store'});if(r.ok&&!draftGoal){await showBlob('goal',await r.blob());seenGoal=key;}}
    text('raw-status',JSON.stringify(s,null,2));drawPath(s);updateButtons();
  } catch(e) {text('connection','连接中断');$('connection').className='pill error';text('mode-notice',`无法读取运行时：${e.message}`);}
}
async function pollImage() {try {const r=await fetch('/api/frame/latest.jpg',{cache:'no-store',signal:AbortSignal.timeout(4000)});if(r.ok) await showBlob('rgb',await r.blob());}catch{}}
async function pollEvents() {try {const body=await api(`/events?after=${lastSequence}`),rows=Array.isArray(body)?body:body.events||[];
  for(const e of rows){lastSequence=Math.max(lastSequence,e.sequence||0);eventRows.push(JSON.stringify(e));}eventRows=eventRows.slice(-100);text('events',eventRows.join('\n')||'暂无事件');
}catch{}}
async function cycle() {await pollStatus();await pollImage();await pollEvents();setTimeout(cycle,500);}
cycle();
