import type { PanelExtensionContext, RenderState } from '@foxglove/extension';
import { initImageGoalPanel } from '../src/panel';
import { TOPICS, record } from '../src/model';
import type { JsonMap } from '../src/model';

const fixture=new URLSearchParams(location.search).get('fixture')??'body';
const demoNotice=document.getElementById('demo-notice');if(demoNotice){demoNotice.textContent=fixture==='live'?'RECORDED SNAPSHOT / Read only · No live robot connection':'DEMO / Simulated data · No robot connection · Controls affect this preview only';}
let epoch=7,paused=false,stopped=false,version=1,selectionVersion=0,taskBackend='nomad',taskMode='imagegoal',taskInstruction='',executionMode='shadow',executorRunning=false;
let selection:JsonMap={mode:'live',ready:false,selection_id:null};
const controls={ackDelayMs:100,suppressTicks:false};
let runtime:JsonMap={};const published:JsonMap[]=[];const subscriptions:{topic:string}[][]=[];
const node=document.getElementById('panel')!;
const context={panelElement:node,initialState:{backend:'nomad',timeout_s:120},dataSourceProfile:'ros2',
  setDefaultPanelTitle:()=>{},saveState:(value:unknown)=>{localStorage.setItem('imagegoal-demo-state',JSON.stringify(value));},
  subscribe:(value:{topic:string}[])=>{subscriptions.push(value);},watch:()=>{},unsubscribeAll:()=>{},unadvertise:()=>{},advertise:()=>{},
  publish:(_topic:string,msg:unknown)=>{
    const value=JSON.parse(String(record(msg).data)) as JsonMap;published.push(value);
    const payload=record(value.payload),action=String(value.action);let ok=true,error:string|undefined,result:unknown;
    if(fixture==='live'){ok=false;error='This is a local read-only preview, with no robot connection.';}
    else if(action==='cancel'||action==='native_stop'){epoch++;stopped=true;paused=false;executorRunning=false;}
    else if(payload.expected_epoch!=undefined&&payload.expected_epoch!==epoch){ok=false;error='expected_epoch mismatch';}
    else if(action==='capture_goal'||action==='import_goal'){
      epoch++;stopped=true;paused=false;selectionVersion++;
      goalData=action==='capture_goal'?rgbData:`data:image/jpeg;base64,${String(record(payload.goal).rgb_jpeg_base64)}`;
      selection={schema_version:'imagegoal.goal_selection.v1',mode:'frozen',ready:true,selection_id:`selection-${selectionVersion}`,sha256:`demo-sha-${selectionVersion}`,source:action==='capture_goal'?'camera':'import',selected_timestamp_ns:Date.now()*1e6};
      result={status:demoStatus(),goal:{rgb_jpeg_base64:goalData.split(',')[1]},selection};
    }
    else if(action==='clear_goal'){epoch++;stopped=true;paused=false;selection={mode:'live',ready:false,selection_id:null};}
    else if((action==='start'||action==='native_start')&&(!payload.task_mode||payload.task_mode==='imagegoal')&&(!selection.ready||payload.selection_id!==selection.selection_id)){ok=false;error='selection_id mismatch';}
    else if(action==='start'||action==='native_start'){epoch++;version=1;stopped=false;paused=false;taskBackend=String(payload.backend);taskMode=String(payload.task_mode??'imagegoal');taskInstruction=String(payload.instruction??'');executionMode=action==='native_start'?'supervised_native':'shadow';executorRunning=action==='native_start';}
    else if(action==='replace_goal'){epoch++;version++;}
    else if(action==='pause'){epoch++;paused=true;}
    else if(action==='resume'){epoch++;paused=false;}
    else if(action==='complete'){epoch++;stopped=true;}
    if(ok&&action!=='capture_goal'&&action!=='import_goal')result=demoStatus();
    setTimeout(()=>emit([{topic:TOPICS.result,message:{data:JSON.stringify({request_id:value.request_id,action,ok,error,result})}}]),controls.ackDelayMs);
  },onRender:undefined,
} as unknown as PanelExtensionContext;
if(fixture==='live'){context.publish=undefined;context.advertise=undefined;}
initImageGoalPanel(context,{snapshot:fixture==='live'});
function emit(messages:{topic:string;message:unknown}[]):void {
  context.onRender?.({colorScheme:new URLSearchParams(location.search).get('theme')==='light'?'light':'dark',currentFrame:messages as RenderState['currentFrame']},()=>{});
}
function demoImage(goal=false):string {
  const c=document.createElement('canvas');c.width=960;c.height=540;const ctx=c.getContext('2d')!;
  const sky=ctx.createLinearGradient(0,0,0,540);sky.addColorStop(0,'#c7cbd0');sky.addColorStop(1,'#74797d');ctx.fillStyle=sky;ctx.fillRect(0,0,960,540);
  const floor=ctx.createLinearGradient(0,220,0,540);floor.addColorStop(0,'#a5a3a1');floor.addColorStop(1,'#77746e');ctx.fillStyle=floor;ctx.beginPath();ctx.moveTo(410,215);ctx.lineTo(565,215);ctx.lineTo(960,540);ctx.lineTo(0,540);ctx.fill();
  ctx.fillStyle='#c6c3bb';ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(410,215);ctx.lineTo(0,540);ctx.fill();ctx.fillStyle='#d6d4cb';ctx.beginPath();ctx.moveTo(960,0);ctx.lineTo(565,215);ctx.lineTo(960,540);ctx.fill();
  for(let i=0;i<5;i++){const t=i/5,x=30+t*330,y=55+t*125,w=160*(1-t)+18,h=370*(1-t)+30;ctx.fillStyle=i%2?'#666860':'#72746b';ctx.fillRect(x,y,w,h);ctx.strokeStyle='#484f49';ctx.lineWidth=4;ctx.strokeRect(x,y,w,h);ctx.fillStyle='#b5bba6';ctx.fillRect(x+w*.7,y+h*.48,5,5);}
  ctx.fillStyle='#3b5144';ctx.fillRect(goal?480:458,206,goal?78:60,110);ctx.fillStyle='#6b8b73';ctx.fillRect(goal?489:465,212,goal?61:46,94);
  ctx.strokeStyle='#bbb7ae';ctx.lineWidth=1;for(let i=0;i<10;i++){ctx.beginPath();ctx.moveTo(488,215);ctx.lineTo(i*110,540);ctx.stroke();}for(let i=1;i<6;i++){const y=230+i*i*9;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(960,y);ctx.stroke();}
  ctx.fillStyle='rgba(12,18,22,.65)';ctx.fillRect(0,0,960,34);ctx.fillStyle='#c7ded4';ctx.font='14px sans-serif';ctx.fillText(goal?'DEMO · FROZEN GOAL':'DEMO · CAMERA PREVIEW',16,23);
  return c.toDataURL('image/jpeg',.85);
}
let rgbData=demoImage(),goalData=demoImage(true);
const bytes=(data:string):Uint8Array=>Uint8Array.from(atob(data.split(',')[1]??''),c=>c.charCodeAt(0));
const image=(data:string,frameId='imagegoal_display_time_preview')=>({format:'jpeg',header:{stamp:{sec:Math.floor(Date.now()/1000),nanosec:0},frame_id:frameId},data:bytes(data)});
function demoStatus():JsonMap {
  const now=Date.now()*1e6,task={task_id:'demo-task-008',backend:taskBackend,task_mode:taskMode,instruction:taskInstruction,goal_version:version,goal_id:`demo-goal-${version}`,execution_mode:executionMode,remaining_s:107,paused};
  const local=[[0,0],[.18,.01],[.36,.05],[.54,.12],[.72,.22],[.88,.34],[1.05,.48],[1.22,.62]];
  return {session_id:'demo-session-20261016',timestamp_ns:now,epoch,profile:'simulated-replay',synthetic_feedback:fixture!=='native',state:stopped?'CANCELLED':paused?'PAUSED':'WAITING_INPUT',task,goal_selection:selection,
    execution_capability:{supervised_native_configured:fixture==='native',operator_supervised:fixture==='native',supervised_native_backends:['nomad','lightnav'],lightnav_imagegoal_adapter:true,task_modes:['imagegoal','objnav','vln','tracking'],supervised_native_task_modes:['imagegoal','objnav','vln','tracking']},native_executor:{running:executorRunning,state:executorRunning?'running':'stopped'},native_execution:{backend:taskBackend,authorized:false},tracking:{enabled:taskMode==='tracking',instruction:taskInstruction,visible:null,target_state:'unknown'},
    gates:fixture==='odom'?[]:['capture_odom_unavailable','fresh_odom_required'],proposal:{vx_m_s:0,wz_rad_s:0},metrics:{inference_accepted:24,inference_rejected:3},
    preview:{epoch,goal_version:version,identity:{session_id:'demo-session-20261016',task_id:task.task_id,goal_id:task.goal_id,goal_version:version},trajectory_body_m:local,source_stamp_ns:now-125e6,distance_steps:6.42,round_trip_ms:94.2},
    plan:fixture==='odom'?{epoch,goal_version:version,odom_frame_id:'go2_odom',source_age_s:.14,trajectory_odom_m_rad:local.map(p=>[p[0]+3,p[1]+2,0]),reference_odom_m_rad:local.map(p=>[p[0]+3,p[1]+2.03,0]),prediction_odom_m_rad:local.map(p=>[p[0]+3.02,p[1]+1.98,0])}:null,
    sensors:{rgb:{age_s:.072,timestamp_source:'simulated_preview'},depth:{age_s:.076,clearance_m:2.34},odom:fixture==='odom'?{age_s:.032,frame_name:'go2_odom',pose_m_rad:[3,2,0]}:null}};
}
async function setup():Promise<void>{
  if(fixture==='live'){
    runtime=await(await fetch('fixture-status.json')).json() as JsonMap;
    for(const [which,path] of [['rgb','live-rgb.jpg'],['goal','live-goal.jpg']]){
      try{const response=await fetch(path!);if(response.ok){const arr=new Uint8Array(await response.arrayBuffer());emit([{topic:which==='rgb'?TOPICS.rgb:TOPICS.goal,message:{format:'jpeg',data:arr}}]);}}catch{/* absent goal is valid */}
    }
  }
  function tick():void {
    const s=fixture==='live'?runtime:demoStatus();emit([{topic:TOPICS.status,message:{data:JSON.stringify(s)}},{topic:TOPICS.bridge,message:{data:JSON.stringify({not_for_motion:true,counters:{sent_rgbd:562,sent_odom:fixture==='odom'?2580:0}})}}]);
    if(fixture!=='live'){
      emit([{topic:TOPICS.rgb,message:image(rgbData)}]);
      if(selection.ready)emit([{topic:TOPICS.goal,message:image(goalData,`imagegoal_frozen_goal_${String(selection.selection_id)}`)}]);
    }
  }
  tick();if(fixture!=='stale'&&fixture!=='live')setInterval(()=>{if(!controls.suppressTicks)tick();},200);
  if(fixture==='live'){
    // Status clears the previous goal image on identity change; refresh image after it.
    setTimeout(async()=>{for(const [topic,path] of [[TOPICS.rgb,'live-rgb.jpg'],[TOPICS.goal,'live-goal.jpg']]){try{const response=await fetch(path!);if(response.ok)emit([{topic:topic!,message:{format:'jpeg',data:new Uint8Array(await response.arrayBuffer())}}]);}catch{/* optional fixture */}}},50);
  }
}
void setup();
Object.assign(globalThis,{imagegoalDemo:{context,published,subscriptions,emit,controls,image,demoStatus,status:()=>fixture==='live'?runtime:demoStatus(),setRgb:(value:string)=>{rgbData=value;},cleanup:()=>node.replaceChildren()}});
