import test from 'node:test';
import assert from 'node:assert/strict';
import {pathView,command,Requests,sourceAgeMs,ACK_TIMEOUT_MS,validateGoalBytes,MAX_GOAL_BYTES,goalSelection,cameraTopic,cameraIdentity,matchesGoalFrame,previewSource,previewTimingLabel,TOPICS,nativeStartReason} from '../src/model.ts';
const status=()=>({session_id:'s1',epoch:4,timestamp_ns:100e9,state:'WAITING_INPUT',goal_selection:{mode:'frozen',ready:true,selection_id:'selection-1',source:'camera'},task:{task_id:'t1',goal_version:2,goal_id:'g1'},preview:{epoch:4,goal_version:2,identity:{session_id:'s1',task_id:'t1',goal_version:2,goal_id:'g1'},source_stamp_ns:99.9e9,trajectory_body_m:[[0,0],[1,.2]]}});
test('body preview works without odom and is explicitly unanchored',()=>{const view=pathView(status());assert.equal(view.mode,'body');assert.match(view.frame,/unanchored/);assert.equal(view.lines.length,1);assert.equal(view.historical,false);});
test('terminal task and cancelled epoch never show retained preview',()=>{for(const modify of [s=>s.state='CANCELLED',s=>s.epoch=5,s=>s.preview.identity.session_id='old',s=>s.preview.goal_version=1,s=>s.preview.identity.task_id='old']){const s=status();modify(s);assert.equal(pathView(s).mode,'empty');}});
test('odom arrays render together and never include mismatched odom pose',()=>{const s=status();s.plan={epoch:4,goal_version:2,odom_frame_id:'odom',trajectory_odom_m_rad:[[2,3,0],[3,4,0]],reference_odom_m_rad:[[2,3,0],[2.9,4,0]],prediction_odom_m_rad:[[2,3,0],[3,3.9,0]]};s.sensors={odom:{frame_name:'other',pose_m_rad:[500,500,0]}};const view=pathView(s);assert.equal(view.mode,'odom');assert.equal(view.lines.length,3);assert.equal(view.pose,undefined);assert.ok(view.lines.every(line=>line.points[0][0]===2));});
test('malformed/nonfinite preview does not create an apparent path',()=>{const s=status();s.preview.trajectory_body_m=[[0,0],[NaN,1]];assert.equal(pathView(s).mode,'empty');});
test('old source output is visibly historical',()=>{const s=status();s.preview.source_stamp_ns=80e9;assert.equal(pathView(s).historical,true);});
test('command uses frozen selection/current epoch and cannot elevate shadow mode or resend raw pixels',()=>{const s=status(),p={backend:'nomad',selection_id:'selection-1',goal:{rgb_jpeg_base64:'jpeg'},timeout_s:120,execution_mode:'operator_run',expected_epoch:1};const c=command('start',s,p,'r1');assert.equal(c.session_id,'s1');assert.equal(c.payload.expected_epoch,4);assert.equal(c.payload.execution_mode,'shadow');assert.equal(c.payload.selection_id,'selection-1');assert.equal(c.payload.goal,undefined);for(const action of ['pause','resume','complete','replace_goal','capture_goal','clear_goal'])assert.equal(command(action,s,{},'x').payload.expected_epoch,4);assert.equal(command('import_goal',s,{goal:{rgb_jpeg_base64:'jpeg'}},'import').payload.expected_epoch,4);assert.equal(command('cancel',s,{},'stop').payload.expected_epoch,undefined);});
test('live/missing/mismatched selection can never start',()=>{for(const selection of [undefined,{mode:'live',ready:false},{mode:'frozen',ready:false,selection_id:'selection-1'},{mode:'frozen',ready:true,selection_id:'different'}]){const s=status();s.goal_selection=selection;assert.throws(()=>command('start',s,{backend:'nomad',selection_id:'selection-1',timeout_s:120},'start'),/[Cc]apture or import/);}});
test('frozen image must carry the exact canonical selection identity',()=>{const s=goalSelection(status());assert.equal(matchesGoalFrame({header:{frame_id:'imagegoal_frozen_goal_selection-1'}},s),true);for(const frame of ['imagegoal_frozen_goal_old','imagegoal_display_time_preview',''])assert.equal(matchesGoalFrame({header:{frame_id:frame}},s),false);assert.equal(matchesGoalFrame({header:{frame_id:'imagegoal_frozen_goal_selection-1'}},goalSelection({})),false);});
test('both camera sources use only the bandwidth-limited RGB topic while provenance stays distinct',()=>{const s=status();assert.equal(cameraTopic(s),TOPICS.rgb);s.sensors={rgb:{frame_name:'go2_front_camera_optical_frame',timestamp_source:'go2_video_rpc_receive_host_system_time_not_sensor_source_time',frame_id:10}};assert.equal(cameraTopic(s),TOPICS.rgb);const identity=cameraIdentity(s);s.sensors.rgb.frame_id=11;assert.equal(cameraIdentity(s),identity);s.sensors.rgb.frame_name='camera_color_optical_frame';s.sensors.rgb.timestamp_source='realsense_ros_header_system_time';assert.notEqual(cameraIdentity(s),identity);assert.equal(cameraTopic(s),TOPICS.rgb);});
test('Go2 preview time label explicitly identifies RPC receive time',()=>{const source=previewSource({header:{frame_id:'go2_front_camera_optical_frame',stamp:{sec:10,nanosec:500000000}}});assert.equal(source.kind,'rpc_receive');assert.equal(previewTimingLabel(source,10600),'RPC receipt time difference 100 ms*');assert.equal(previewSource({header:{frame_id:'imagegoal_display_time_preview',stamp:{sec:10,nsec:0}}}).kind,'display');});
test('invalid timeout rejected before publish',()=>{assert.throws(()=>command('start',status(),{backend:'nomad',goal:{rgb_jpeg_base64:'x'},timeout_s:Infinity},'r1'),/Timeout/);});
test('stop bypasses pending start and supersedes its eventual ack',()=>{const r=new Requests();r.begin('start','start',1);assert.throws(()=>r.begin('next','pause',2));r.begin('stop','cancel',3);assert.equal(r.stopping,true);assert.equal(r.settle('start').superseded,true);assert.equal(r.settle('stop').superseded,false);assert.equal(r.busy,false);});
test('pending requests expire and session reset forgets old acks',()=>{const r=new Requests();r.begin('x','capture_goal',0);assert.equal(r.expired(ACK_TIMEOUT_MS+1).length,1);r.reset();assert.equal(r.settle('x'),undefined);});
test('source age increases after the last status arrival',()=>{assert.equal(sourceAgeMs({age_s:.08},120),200);assert.equal(sourceAgeMs(null),undefined);});

test('native start keeps frozen identity, selected backend/instruction, fixed budget and supervision',()=>{
  const s=status(),input={backend:'nomad',selection_id:'selection-1',timeout_s:3600,operator_present:false,goal:{rgb_jpeg_base64:'not-resubmitted'}};
  assert.throws(()=>command('native_start',s,input,'r'),/[Ss]upervised-native/);
  s.execution_capability={supervised_native_configured:true,operator_supervised:true,supervised_native_backends:['nomad','lightnav']};
  for(const backend of ['nomad']){
    const native=command('native_start',s,{...input,backend,instruction:'到参考图的位置'},'r');
    assert.deepEqual(native.payload,{expected_session_id:'s1',expected_epoch:4,selection_id:'selection-1',backend,instruction:backend==='lightnav'?'到参考图的位置':'',timeout_s:60,operator_present:true});
    assert.equal(command('native_start',s,{...input,backend},'r').payload.instruction,'');
  }
  assert.throws(()=>command('native_start',s,{...input,backend:'unknown'},'r'),/supported navigation policy/);
  assert.throws(()=>command('native_start',s,{...input,selection_id:'old'},'r'),/goal has changed/);
  s.execution_capability.operator_supervised=false;assert.match(nativeStartReason(s,'nomad'),/supervision/);
});
test('native capabilities retain NoMaD compatibility and require explicit official LightNav modes',()=>{
  const s=status();s.execution_capability={supervised_native_configured:true,operator_supervised:true};
  assert.equal(nativeStartReason(s,'nomad'),'');assert.match(nativeStartReason(s,'lightnav'),/LightNav.*unavailable/);
  assert.match(nativeStartReason(s,'lightnav','objnav'),/LightNav.*unavailable/);
  s.execution_capability.supervised_native_backends=['nomad','lightnav'];
  assert.match(nativeStartReason(s,'lightnav','objnav'),/Official LightNav navigation.*unavailable/);
  s.execution_capability.supervised_native_task_modes=['objnav','vln'];s.goal_selection={mode:'live',ready:false};
  assert.equal(nativeStartReason(s,'lightnav','objnav'),'');assert.equal(nativeStartReason(s,'lightnav','vln'),'');
  s.native_executor={running:true};assert.match(nativeStartReason(s,'lightnav','objnav'),/executor is running/);
});
test('native stop bypasses pending start, supersedes it and does not wait for its late acknowledgement',()=>{
  const r=new Requests();r.begin('native-start','native_start',0);r.begin('stop','native_stop',1);assert.equal(r.stopping,true);r.settle('stop');assert.equal(r.busy,false);assert.equal(r.stopping,false);assert.equal(r.settle('native-start').superseded,true);
  assert.deepEqual(command('native_stop',status(),{expected_epoch:999},'stop').payload,{expected_session_id:'s1',reason:'operator_stop'});
});

test('goal upload enforces runtime 4 MiB JPEG limit',()=>{const tooLarge=new Uint8Array(MAX_GOAL_BYTES+1);tooLarge.set([255,216]);assert.throws(()=>validateGoalBytes(tooLarge),/4 MB/);assert.throws(()=>validateGoalBytes(new Uint8Array([137,80,78,71])),/JPEG/);assert.doesNotThrow(()=>validateGoalBytes(new Uint8Array([255,216,255])));});

const trackingStatus=()=>({...status(),goal_selection:{mode:'live',ready:false},execution_capability:{supervised_native_configured:true,operator_supervised:true,supervised_native_backends:['nomad','lightnav'],task_modes:['imagegoal','objnav','vln','tracking'],supervised_native_task_modes:['imagegoal','objnav','vln','tracking']}});
test('native tracking requires no image and preserves session/supervision with an explicit tracking task',()=>{
  const s=trackingStatus(),input={backend:'lightnav',task_mode:'tracking',instruction:'  Follow the person in white.  ',selection_id:'old-image',goal:{rgb_jpeg_base64:'must-not-send'},timeout_s:3600,operator_present:false};
  assert.deepEqual(command('native_start',s,input,'track').payload,{expected_session_id:'s1',expected_epoch:4,backend:'lightnav',task_mode:'tracking',instruction:'Follow the person in white.',timeout_s:60,operator_present:true});
  assert.deepEqual(command('start',s,{...input,timeout_s:120,execution_mode:'supervised_native'},'shadow').payload,{backend:'lightnav',task_mode:'tracking',timeout_s:120,instruction:'Follow the person in white.',expected_epoch:4,execution_mode:'shadow'});
});
test('tracking rejects old runtimes, non-LightNav backends, empty instructions, active executors and unknown task modes',()=>{
  const input={backend:'lightnav',task_mode:'tracking',instruction:'Follow the person.',timeout_s:120};
  for(const action of ['start','native_start']){
    for(const instruction of ['', '   ',null,'x'.repeat(4097)])assert.throws(()=>command(action,trackingStatus(),{...input,instruction},'r'),/[Tt]arget|description/);
    assert.throws(()=>command(action,trackingStatus(),{...input,backend:'nomad'},'r'),/requires LightNav/);
    assert.throws(()=>command(action,trackingStatus(),{...input,task_mode:'unsupported'},'r'),/supported task mode/);
    const old=trackingStatus();delete old.execution_capability.task_modes;delete old.execution_capability.supervised_native_task_modes;assert.throws(()=>command(action,old,input,'r'),/LightNav tracking.*unavailable/);
  }
  const s=trackingStatus();s.native_executor={running:true};assert.throws(()=>command('native_start',s,input,'r'),/executor is running/);
  s.native_executor.running=false;s.execution_capability.operator_supervised=false;assert.throws(()=>command('native_start',s,input,'r'),/supervision/);
});

test('official LightNav navigation uses only required language and explicit ObjNav/VLN modes',()=>{
  for(const task_mode of ['objnav','vln']){
    const s=trackingStatus(),input={backend:'lightnav',task_mode,instruction:'  Go to the coat rack.  ',selection_id:'old-image',goal:{rgb_jpeg_base64:'must-not-send'},timeout_s:120};
    assert.deepEqual(command('start',s,input,'nav').payload,{backend:'lightnav',task_mode,timeout_s:120,instruction:'Go to the coat rack.',expected_epoch:4,execution_mode:'shadow'});
    assert.deepEqual(command('native_start',s,input,'nav').payload,{expected_session_id:'s1',expected_epoch:4,backend:'lightnav',task_mode,instruction:'Go to the coat rack.',timeout_s:60,operator_present:true});
    for(const action of ['start','native_start']){
      for(const instruction of ['', '   ',null,'x'.repeat(4097)])assert.throws(()=>command(action,s,{...input,instruction},'r'),/[Ii]nstruction/);
      assert.throws(()=>command(action,s,{...input,backend:'nomad'},'r'),/requires LightNav/);
      const old=trackingStatus();delete old.execution_capability.task_modes;delete old.execution_capability.supervised_native_task_modes;
      assert.throws(()=>command(action,old,input,'r'),/Official LightNav navigation.*unavailable/);
    }
  }
});
test('LightNav task mode must remain explicit and ImageGoal adaptation requires its capability',()=>{
  for(const action of ['start','native_start']){
    const input={backend:'lightnav',selection_id:'selection-1',instruction:'',timeout_s:120};
    assert.throws(()=>command(action,trackingStatus(),input,'legacy'),/explicit LightNav task mode/);
    assert.throws(()=>command(action,trackingStatus(),{...input,task_mode:'imagegoal'},'adapt'),/LightNav ImageGoal.*unavailable/);
  }
});
test('LightNav ImageGoal adaptation requires its frozen image and preserves optional text separately',()=>{
  const s=trackingStatus();s.goal_selection=status().goal_selection;s.execution_capability.lightnav_imagegoal_adapter=true;
  const input={backend:'lightnav',task_mode:'imagegoal',selection_id:'selection-1',instruction:'  Approach the goal view.  ',timeout_s:120,goal:{rgb_jpeg_base64:'must-not-resend'}};
  assert.deepEqual(command('start',s,input,'adapt').payload,{selection_id:'selection-1',backend:'lightnav',task_mode:'imagegoal',timeout_s:120,instruction:'Approach the goal view.',expected_epoch:4,execution_mode:'shadow'});
  assert.deepEqual(command('native_start',s,input,'adapt').payload,{expected_session_id:'s1',expected_epoch:4,selection_id:'selection-1',backend:'lightnav',task_mode:'imagegoal',instruction:'Approach the goal view.',timeout_s:60,operator_present:true});
  for(const action of ['start','native_start']){
    assert.equal(command(action,s,{...input,instruction:undefined},'optional').payload.instruction,'');
    const missing={...s,goal_selection:{mode:'live',ready:false}};assert.throws(()=>command(action,missing,input,'missing'),/[Cc]apture or import/);
    assert.throws(()=>command(action,s,{...input,instruction:'x'.repeat(4097)},'long'),/Additional instructions/);
  }
});
