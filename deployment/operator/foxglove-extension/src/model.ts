export type JsonMap = Record<string, unknown>;
export type Action = 'start' | 'native_start' | 'native_stop' | 'replace_goal' | 'pause' | 'resume' | 'cancel' | 'complete' | 'capture_goal' | 'import_goal' | 'clear_goal';
export type TaskMode = 'imagegoal' | 'objnav' | 'vln' | 'tracking';
export const DEFAULT_TRACKING_INSTRUCTION = 'Follow the person in the white T-shirt.';
export type Point = [number, number, ...number[]];
export const TOPICS = {
  status: '/imagegoal/shadow/status', bridge: '/imagegoal/shadow/bridge_status',
  result: '/imagegoal/operator/result', rgb: '/imagegoal/shadow/rgb/compressed',
  goal: '/imagegoal/shadow/goal/compressed', command: '/imagegoal/operator/command',
} as const;
export const TERMINAL = new Set(['IDLE', 'CANCELLED', 'SUCCEEDED', 'FAILED', 'COMPLETED', 'TIMED_OUT']);
export const STATUS_STALE_MS = 2500;
export const ACK_TIMEOUT_MS = 10000;
export const MAX_GOAL_BYTES = 4*1024*1024;
export function validateGoalBytes(bytes:Uint8Array):void {
  if(bytes.byteLength>MAX_GOAL_BYTES||bytes[0]!==255||bytes[1]!==216){throw new Error('Choose a JPEG goal image no larger than 4 MB.');}
}
export const record = (v: unknown): JsonMap => v != undefined && typeof v === 'object' && !Array.isArray(v) ? v as JsonMap : {};
export const number = (v: unknown): number | undefined => typeof v === 'number' && Number.isFinite(v) ? v : undefined;
export const str = (v: unknown, fallback = '—'): string => typeof v === 'string' && v.length > 0 ? v : fallback;
export const metric = (v: unknown, unit = '', digits = 1): string => number(v) == undefined ? '—' : `${(v as number).toFixed(digits)}${unit}`;
export const active = (status: JsonMap): boolean => Object.keys(record(status.task)).length > 0 && !TERMINAL.has(str(status.state).toUpperCase());
export function sourceAgeMs(sensor: unknown, elapsedMs = 0): number | undefined {
  const value = record(sensor); const age = number(value.age_ms) ?? (number(value.age_s) == undefined ? undefined : (value.age_s as number) * 1000);
  return age == undefined ? undefined : age + Math.max(0, elapsedMs);
}
export interface GoalSelection { mode:'live'|'frozen'; ready:boolean; id:string; source:string; sha256:string; }
export function goalSelection(status:JsonMap):GoalSelection {
  const selection=record(status.goal_selection),id=str(selection.selection_id,'');
  const frozen=selection.mode==='frozen'&&selection.ready===true&&id.length>0;
  return {mode:frozen?'frozen':'live',ready:frozen,id:frozen?id:'',source:str(selection.source,''),sha256:str(selection.sha256,'')};
}
export function cameraIdentity(status:JsonMap):string {
  const sensor=record(record(status.sensors).rgb);
  return `${str(sensor.frame_name,'')}/${str(sensor.timestamp_source,'')}`;
}
export function cameraTopic(_status:JsonMap):typeof TOPICS.rgb {
  // Source provenance stays in the original header/status. Never subscribe to
  // the full-resolution native stream just to identify which camera is active.
  return TOPICS.rgb;
}
export function matchesGoalFrame(message:unknown,selection:GoalSelection):boolean {
  return selection.ready&&record(record(message).header).frame_id===`imagegoal_frozen_goal_${selection.id}`;
}
export interface PreviewSource { kind:'original'|'rpc_receive'|'display'|'unknown'; stampMs?:number; frameId:string; }
export function previewSource(message:unknown):PreviewSource {
  const header=record(record(message).header),frameId=str(header.frame_id,''),stamp=record(header.stamp);
  // Older bridge previews stamped publication/display time, never camera capture time.
  if(frameId.includes('display_time_preview')){return {kind:'display',frameId};}
  const sec=number(stamp.sec),nsec=number(stamp.nanosec)??number(stamp.nsec);
  if(sec==undefined||nsec==undefined||!Number.isInteger(sec)||!Number.isInteger(nsec)||sec<0||nsec<0||nsec>=1e9||sec*1e9+nsec<=0){return {kind:'unknown',frameId};}
  return {kind:frameId.startsWith('go2_front_camera')?'rpc_receive':'original',stampMs:sec*1000+nsec/1e6,frameId};
}
export function previewTimingLabel(source:PreviewSource,nowMs:number):string {
  if(source.kind==='display'){return 'Legacy display stamp · source latency unknown';}
  if(source.kind==='unknown'||source.stampMs==undefined){return 'Preview source time unavailable';}
  const difference=nowMs-source.stampMs;
  const label=source.kind==='rpc_receive'?'RPC receipt time':'Preview source time';
  if(difference<0){return `${label} ahead by ${metric(-difference,' ms',0)} · check clock offset`;}
  return `${label} difference ${metric(difference,' ms',0)}*`;
}
export function points(value: unknown): Point[] {
  if (!Array.isArray(value) || value.length === 0 || value.length > 10000) {return [];}
  if (!value.every(p => Array.isArray(p) && p.length >= 2 && p.every(n => number(n) != undefined))) {return [];}
  return value as Point[];
}
export interface PathView {
  mode: 'empty' | 'body' | 'odom'; frame: string; note: string; historical: boolean;
  lines: { label: string; color: string; points: Point[] }[]; pose?: Point; sourceAgeS?: number;
}
export function pathView(status: JsonMap): PathView {
  const empty = (note: string): PathView => ({mode:'empty',frame:'Awaiting policy',note,historical:false,lines:[]});
  if (!active(status)) {return empty(TERMINAL.has(str(status.state).toUpperCase()) && Object.keys(record(status.task)).length>0 ? 'No active task · previous predictions hidden' : 'Start a task to view its trajectory');}
  const task = record(status.task), plan = record(status.plan), preview = record(status.preview), identity = record(preview.identity);
  const current = (item: JsonMap): boolean => number(item.epoch) === number(status.epoch) && number(item.goal_version) === number(task.goal_version);
  const sourceAge = (item: JsonMap): number | undefined => number(item.source_age_s) ??
    (number(status.timestamp_ns) != undefined && number(item.source_stamp_ns) != undefined ? ((status.timestamp_ns as number) - (item.source_stamp_ns as number)) / 1e9 : undefined);
  const trajectory = points(plan.trajectory_odom_m_rad);
  if (trajectory.length>0 && current(plan) && typeof plan.odom_frame_id === 'string' && plan.odom_frame_id.length > 0) {
    const frame = plan.odom_frame_id, odom = record(record(status.sensors).odom);
    const age = sourceAge(plan);
    const pose = odom.frame_name === frame ? points([odom.pose_m_rad])[0] : undefined;
    return {mode:'odom', frame, note:'Odometry frame · policy and MPC share the same reference frame', historical:(age ?? 0) > 2, sourceAgeS:age, pose,
      lines:[{label:'Policy path',color:'var(--trajectory)',points:trajectory},{label:'MPC reference',color:'var(--reference)',points:points(plan.reference_odom_m_rad)},
        {label:'MPC prediction',color:'var(--prediction)',points:points(plan.prediction_odom_m_rad)}]};
  }
  if (points(preview.trajectory_body_m).length>0 && current(preview) && identity.session_id === status.session_id &&
      identity.task_id === task.task_id && identity.goal_version === task.goal_version && identity.goal_id === task.goal_id) {
    const age = sourceAge(preview);
    return {mode:'body',frame:'Body frame · unanchored',note:'↑ Forward +x · ← Left +y · robot origin at image capture',historical:(age ?? 0)>2,sourceAgeS:age,
      pose:[0,0,0],lines:[{label:'Local policy preview',color:'var(--trajectory)',points:points(preview.trajectory_body_m)}]};
  }
  return empty(Object.keys(preview).length>0 || Object.keys(plan).length>0 ? 'Obsolete or invalid predictions hidden · awaiting current output' : 'Awaiting policy output · local preview is available without odometry');
}
export interface Command { request_id: string; session_id: string; action: Action; payload: JsonMap; }
export function trackingStartReason(status:JsonMap,backend:string,instruction:unknown,_taskMode='tracking'):string {
  if(backend!=='lightnav'){return 'Tracking requires LightNav.';}
  const supported=record(status.execution_capability).task_modes;
  if(!Array.isArray(supported)||!supported.includes('tracking')){return 'LightNav tracking is unavailable. Update or restart the runtime.';}
  if(typeof instruction!=='string'||!instruction.trim()){return 'Enter a target description, for example a person in a white T-shirt.';}
  if(instruction.length>4096){return 'Target descriptions must be at most 4096 characters.';}
  return '';
}
export function navigationStartReason(status:JsonMap,backend:string,instruction:unknown,taskMode='objnav'):string {
  if(backend!=='lightnav'){return 'Official language navigation requires LightNav.';}
  const supported=record(status.execution_capability).task_modes;
  if(!Array.isArray(supported)||!supported.includes(taskMode)){return 'Official LightNav navigation is unavailable. Update or restart the runtime.';}
  if(typeof instruction!=='string'||!instruction.trim()){return 'Enter a navigation instruction, for example: Go to the coat rack beside the windows.';}
  if(instruction.length>4096){return 'Navigation instructions must be at most 4096 characters.';}
  return '';
}
export function imageGoalStartReason(status:JsonMap,backend:string,instruction:unknown):string {
  if(backend!=='lightnav'){return '';}
  const capability=record(status.execution_capability);
  if(capability.lightnav_imagegoal_adapter!==true||!Array.isArray(capability.task_modes)||!capability.task_modes.includes('imagegoal')){return 'LightNav ImageGoal adaptation is unavailable. Update or restart the runtime.';}
  if(instruction!=undefined&&(typeof instruction!=='string'||instruction.length>4096)){return 'Additional instructions must be text of at most 4096 characters.';}
  return '';
}
export function nativeStartReason(status:JsonMap,backend:string,taskMode='imagegoal'):string {
  const capability=record(status.execution_capability);
  if(!['nomad','lightnav'].includes(backend)){return 'Choose a supported navigation policy.';}
  if(!['imagegoal','objnav','vln','tracking'].includes(taskMode)){return 'Choose a supported task mode.';}
  const supported=capability.supervised_native_backends;
  if(Array.isArray(supported)?!supported.includes(backend):backend!=='nomad'){return `${backend==='lightnav'?'LightNav':'NoMaD'} robot execution is unavailable. Update or restart the runtime.`;}
  if(capability.supervised_native_configured!==true||capability.operator_supervised!==true){return 'Supervised-native execution and operator supervision are not configured.';}
  if(record(status.native_executor).running===true){return 'The robot executor is running. Stop it before starting a new run.';}
  if(taskMode==='tracking'){
    if(backend!=='lightnav'){return 'Tracking requires LightNav.';}
    if(!Array.isArray(capability.supervised_native_task_modes)||!capability.supervised_native_task_modes.includes('tracking')){return 'LightNav tracking on robot is unavailable. Update or restart the runtime.';}
  }else if(taskMode==='objnav'||taskMode==='vln'){
    if(backend!=='lightnav'){return 'Official language navigation requires LightNav.';}
    if(!Array.isArray(capability.supervised_native_task_modes)||!capability.supervised_native_task_modes.includes(taskMode)){return 'Official LightNav navigation on robot is unavailable. Update or restart the runtime.';}
  }else{
    if(backend==='lightnav'){
      const reason=imageGoalStartReason(status,backend,'');if(reason){return reason;}
      if(!Array.isArray(capability.supervised_native_task_modes)||!capability.supervised_native_task_modes.includes('imagegoal')){return 'LightNav ImageGoal robot adaptation is unavailable. Update or restart the runtime.';}
    }
    if(!goalSelection(status).ready){return 'Capture or import a full goal image first.';}
  }
  return '';
}
export const stopAction=(action:Action):boolean=>action==='cancel'||action==='native_stop';
export function command(action: Action, status: JsonMap, input: JsonMap, requestId: string): Command {
  let payload=input;
  if (typeof status.session_id !== 'string' || number(status.epoch) == undefined) {throw new Error('A valid runtime session is not available.');}
  if (['start','replace_goal','pause','resume','complete','capture_goal','import_goal','clear_goal'].includes(action)) {payload = {...payload,expected_epoch:status.epoch};}
  if (action === 'start') {
    if (!['nomad','lightnav'].includes(str(payload.backend))) {throw new Error('Choose a supported policy.');}
    if (number(payload.timeout_s) == undefined || (payload.timeout_s as number) < 5 || (payload.timeout_s as number) > 3600) {throw new Error('Timeout must be between 5 and 3600 seconds.');}
    if(payload.task_mode!=undefined&&!['imagegoal','objnav','vln','tracking'].includes(str(payload.task_mode))){throw new Error('Choose a supported task mode.');}
    if(payload.backend==='lightnav'&&payload.task_mode==undefined){throw new Error('Choose an explicit LightNav task mode.');}
    if(payload.task_mode==='tracking'||payload.task_mode==='objnav'||payload.task_mode==='vln'){
      const reason=(payload.task_mode==='tracking'?trackingStartReason:navigationStartReason)(status,str(payload.backend),payload.instruction,str(payload.task_mode));if(reason){throw new Error(reason);}
      payload={backend:'lightnav',task_mode:payload.task_mode,timeout_s:payload.timeout_s,instruction:str(payload.instruction,'').trim(),expected_epoch:status.epoch,execution_mode:'shadow'};
    }else{
      const reason=imageGoalStartReason(status,str(payload.backend),payload.instruction);if(reason){throw new Error(reason);}
      const selection=goalSelection(status);
      if (!selection.ready||payload.selection_id!==selection.id) {throw new Error('Capture or import a goal and wait for runtime confirmation.');}
      payload = {selection_id:selection.id,backend:payload.backend,...(payload.backend==='lightnav'?{task_mode:'imagegoal'}:{}),timeout_s:payload.timeout_s,instruction:payload.backend==='lightnav'?str(payload.instruction,'').trim():'',expected_epoch:status.epoch,execution_mode:'shadow'};
    }
  }
  if(action==='native_start'){
    if(payload.backend==='lightnav'&&payload.task_mode==undefined){throw new Error('Choose an explicit LightNav task mode.');}
    const taskMode=str(payload.task_mode,'imagegoal'),reason=nativeStartReason(status,str(payload.backend),taskMode);if(reason){throw new Error(reason);}
    if(taskMode==='tracking'||taskMode==='objnav'||taskMode==='vln'){
      const instructionReason=(taskMode==='tracking'?trackingStartReason:navigationStartReason)(status,str(payload.backend),payload.instruction,taskMode);if(instructionReason){throw new Error(instructionReason);}
      payload={expected_session_id:status.session_id,expected_epoch:status.epoch,backend:'lightnav',task_mode:taskMode,instruction:str(payload.instruction,'').trim(),timeout_s:60,operator_present:true};
    }else{
      const imageGoalReason=imageGoalStartReason(status,str(payload.backend),payload.instruction);if(imageGoalReason){throw new Error(imageGoalReason);}
      const selection=goalSelection(status);if(payload.selection_id!==selection.id){throw new Error('The goal has changed. Wait for the current frozen goal.');}
      payload={expected_session_id:status.session_id,expected_epoch:status.epoch,selection_id:selection.id,backend:payload.backend,...(payload.backend==='lightnav'?{task_mode:'imagegoal'}:{}),instruction:payload.backend==='lightnav'?str(payload.instruction,'').trim():'',timeout_s:60,operator_present:true};
    }
  }
  if(action==='native_stop'){payload={expected_session_id:status.session_id,reason:'operator_stop'};}
  if (action==='import_goal') {
    if(typeof record(payload.goal).rgb_jpeg_base64!=='string'||!str(record(payload.goal).rgb_jpeg_base64,'')) {throw new Error('The import request is missing JPEG data.');}
  }
  return {request_id:requestId,session_id:status.session_id,action,payload};
}
export interface Pending { id: string; action: Action; startedAt: number; generation: number; }
export class Requests {
  pending = new Map<string, Pending>(); generation = 0;
  get busy(): boolean { return [...this.pending.values()].some(p => p.generation===this.generation&&!stopAction(p.action)); }
  get stopping(): boolean { return [...this.pending.values()].some(p => p.generation===this.generation&&stopAction(p.action)); }
  begin(id: string, action: Action, now: number): Pending {
    if (stopAction(action)) {this.generation++;}
    else if (this.busy || this.stopping) {throw new Error('An action is already awaiting acknowledgement.');}
    const value = {id,action,startedAt:now,generation:this.generation}; this.pending.set(id,value); return value;
  }
  settle(id: string): (Pending & {superseded:boolean}) | undefined {
    const value = this.pending.get(id); if (!value) {return undefined;}
    this.pending.delete(id); return {...value,superseded:value.generation !== this.generation};
  }
  expired(now: number): Pending[] { return [...this.pending.values()].filter(v => now-v.startedAt > ACK_TIMEOUT_MS); }
  reset(): void { this.pending.clear(); this.generation++; }
}
export function gates(status: JsonMap): string[] {
  const all = [status.gates,record(status.execution_envelope).gate_snapshot];
  const values: string[] = [];
  for (const group of all) {
    if (Array.isArray(group)) {values.push(...group.map(v => typeof v === 'string' ? v : JSON.stringify(v)));}
    else {for (const [key,value] of Object.entries(record(group))) {if (value !== true) {values.push(`${key}: ${String(value)}`);}}}
  }
  return [...new Set(values)];
}

// Presentation only. Status values and machine-readable gate codes are retained
// verbatim in diagnostics; this never changes command or capability decisions.
const REASON_LABELS: Record<string,string> = {
  odom_missing:'Odometry is not available', fresh_odom_required:'Waiting for fresh odometry',
  capture_odom_unavailable:'Capture has no odometry anchor', fresh_plan_required:'Waiting for a fresh plan',
  rgb_missing:'Camera input is not available', rgb_stale:'Camera input is stale',
  odom_stale:'Odometry is stale', depth_missing:'Depth input is not available', depth_stale:'Depth input is stale',
  target_not_visible:'Target not visible', model_stop:'Policy STOP', zero_trajectory:'Zero trajectory',
  session_capacity:'Model session capacity reached', task_retired:'This model session has been closed',
  inference_busy:'Model inference is busy', server_restarted:'Model service restarted',
  history_capacity:'Observation history limit reached', history_conflict:'Observation history does not match',
  lightnav_transport_failed_no_image_retry:'Model connection failed; start a new task after reconnecting',
  lightnav_pending_frames_exceeded:'Pending observation limit reached',
  lightnav_pending_age_exceeded:'Pending observations are too old',
};
export function displayReason(value:string):string {
  if(REASON_LABELS[value]){return REASON_LABELS[value];}
  if(/\p{Script=Han}/u.test(value)){return 'Runtime message available in Diagnostics.';}
  return value;
}
