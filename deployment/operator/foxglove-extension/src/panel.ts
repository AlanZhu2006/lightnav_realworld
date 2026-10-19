import type { PanelExtensionContext } from '@foxglove/extension';

import { DEFAULT_TRACKING_INSTRUCTION, TOPICS, STATUS_STALE_MS, Requests, active, cameraIdentity, cameraTopic, command, displayReason, gates, goalSelection, imageGoalStartReason, matchesGoalFrame, metric, nativeStartReason, navigationStartReason, number, pathView, previewSource, previewTimingLabel, record, sourceAgeMs, stopAction, str, trackingStartReason, validateGoalBytes } from './model';
import type { Action, JsonMap, PathView, Point, PreviewSource, TaskMode } from './model';
import { styles } from './styles';

// std_msgs/String has one primitive string field; no runtime schema dependency is required.
const STRING_SCHEMA={name:'std_msgs/String',definitions:[{name:'data',type:'string',isComplex:false,isArray:false}]};

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className = '', text?: string): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag); node.className = className; if (text != undefined) {node.textContent = text;} return node;
}
function card(title: string, meta: HTMLElement = el('span')): {box:HTMLDivElement;body:HTMLDivElement;head:HTMLDivElement} {
  const box = el('div','ig-card'), head = el('div','ig-cardhead'), body = el('div','ig-body'); head.append(el('h2','',title),meta); box.append(head,body); return {box,head,body};
}
function button(text: string, action: string, className = ''): HTMLButtonElement {
  const b = el('button',className,text); b.type = 'button'; b.dataset.testid = action; return b;
}
function parseString(message: unknown): JsonMap | undefined {
  try { const data = record(message).data; if (typeof data === 'string') {return record(JSON.parse(data));} } catch { /* malformed data is displayed as an error, never executed */ }
  return undefined;
}
function compressedBytes(message: unknown): Uint8Array | undefined {
  const data = record(message).data;
  if (data instanceof Uint8Array) {return new Uint8Array(data);}
  if (Array.isArray(data) && data.length <= 12*1024*1024 && data.every(v => typeof v === 'number' && v >= 0 && v <= 255)) {return new Uint8Array(data);}
  if (typeof data === 'string' && data.length < 17*1024*1024) { try { return Uint8Array.from(atob(data),c=>c.charCodeAt(0)); } catch { return undefined; } }
  return undefined;
}
function displayURL(bytes: Uint8Array): string { return URL.createObjectURL(new Blob([new Uint8Array(bytes).buffer],{type:'image/jpeg'})); }
const STATE_NAMES: Record<string,string> = {IDLE:'Ready',CANCELLED:'Stopped',SUCCEEDED:'Arrival confirmed',FAILED:'Failed',TIMED_OUT:'Timed out',PAUSED:'Paused',WAITING_INPUT:'Waiting for input',TRACKING:'Tracking',TRACK_HOLD:'Tracking held · observing',HOLDING:'On hold',COMPLETED:'Completed'};
const ACTION_NAMES: Record<Action,string> = {start:'Start task',native_start:'Run on robot',native_stop:'Stop robot',replace_goal:'Replace goal',pause:'Pause',resume:'Resume',cancel:'Stop',complete:'Confirm arrival',capture_goal:'Capture goal',import_goal:'Import goal',clear_goal:'Reset goal'};

export function initImageGoalPanel(context: PanelExtensionContext, options: {snapshot?: boolean} = {}): () => void {
  const snapshot=options.snapshot===true;
  const root = el('div','ig-app'), style = el('style'); style.textContent = styles; root.append(style); context.panelElement.append(root);
  root.dataset.theme='dark';
  root.lang='en';
  context.setDefaultPanelTitle('Navigation Console');
  const initial=record(context.initialState); let backend = initial.backend === 'lightnav' ? 'lightnav' : 'nomad';
  let taskMode:TaskMode=backend==='lightnav'?(initial.task_mode==='tracking'?'tracking':initial.task_mode==='vln'?'vln':initial.task_mode==='imagegoal'?'imagegoal':'objnav'):'imagegoal';
  let imageGoalInstruction=str(initial.imagegoal_instruction,''),navigationInstruction=str(initial.navigation_instruction,''),trackingInstruction=str(initial.tracking_instruction,DEFAULT_TRACKING_INSTRUCTION);
  let status:JsonMap={}, bridge:JsonMap={}, receivedAt=0, bridgeAt=0, rgbAt=0, goalAt=0;
  let disposed=false, advertised=false, goalIdentity='', rgbURL:string|undefined,goalURL:string|undefined, uploadGeneration=0;
  let goalImageReady=false,validatingUpload=false,displayGeneration=0;
  let subscribedGoal:boolean|undefined;
  let rgbGeneration=0,rgbLoading=false,goalLoadingId='',liveSyncURL='';
  let pendingRgb:{bytes:Uint8Array;source:PreviewSource;receivedAt:number}|undefined;
  const ownedURLs=new Set<string>();
  let rgbSource:PreviewSource={kind:'unknown',frameId:''};
  let lastRuntimeMessage='';
  let uncertainty=false;let waitingEpoch:number|undefined; const requests = new Requests(); const logRows:string[]=[];
  const head=el('header','ig-head'),brand=el('div','ig-brand'),wordmark=el('div');
  wordmark.append(el('div','ig-kicker','DEJAVIEW  /  GO2'),el('h1','','Navigation Console')); brand.append(el('div','ig-mark','◈'),wordmark);
  const badges=el('div','ig-badges'),connection=el('span','ig-pill','Connecting'),stateBadge=el('span','ig-pill','IDLE');
  connection.dataset.testid='connection'; badges.append(connection,stateBadge); head.append(brand,badges); root.append(head);
  const banner=el('div','ig-banner'),bannerText=el('span','','Shadow mode previews the policy and controller without robot execution.'),bannerMode=el('strong','','SHADOW · PREVIEW');
  banner.append(bannerMode,bannerText);root.append(banner);
  const layout=el('div','ig-layout'),main=el('main','ig-main'),side=el('aside','ig-side');layout.append(main,side);root.append(layout);
  const pictures=el('div','ig-pictures');main.append(pictures);
  function imageCard(title:string,testid:string): {box:HTMLDivElement;img:HTMLImageElement;area:HTMLDivElement;meta:HTMLSpanElement;footer:HTMLDivElement} {
    const meta=el('span','ig-meta','Waiting for image'),c=card(title,meta),img=el('img'),area=el('div','ig-imagebox'),footer=el('div','ig-imagefooter');
    img.alt=title;img.decoding='async';img.dataset.testid=testid;area.append(img,el('div','ig-empty',testid==='rgb'?'Waiting for camera\nConnect a Foxglove WebSocket source':'Waiting for camera\nCapture or import an image to set the goal'));
    c.body.remove();c.box.append(area,footer);pictures.append(c.box);return {box:c.box,img,area,meta,footer};
  }
  const camera=imageCard('Live camera','rgb'),goal=imageCard('Goal image · live preview','goal');
  const goalMode=el('span','ig-imagestate','Live preview · no goal captured');goalMode.dataset.testid='goal-mode';goal.area.append(goalMode);
  const cameraFoot=el('span','','Preview source time unavailable');cameraFoot.dataset.testid='preview-source-age';camera.meta.dataset.testid='preview-receipt-age';camera.footer.append(cameraFoot);
  const goalFoot=el('span','', 'Capture or import a goal to begin'),upload=button('Import JPEG','upload'),capture=button('Capture frame','capture'),clear=button('Reset goal','clear_goal');
  capture.title='Freeze the latest original inference RGB frame as the goal. This does not start a task; the frame may differ from the preview by one sample.';
  clear.title='Stop the current task, clear the frozen goal, and return to live preview.';
  const goalActions=el('div','ig-goalactions');goalActions.append(upload,capture,clear);goal.footer.append(goalFoot,goalActions);
  const file=el('input','ig-file');file.type='file';file.accept='image/jpeg';file.dataset.testid='goal-file';root.append(file);
  const pathLabel=el('span','ig-meta','Awaiting policy'),pathCard=card('Trajectory',pathLabel),pathWrap=el('div','ig-pathwrap'),canvas=el('canvas','ig-path');
  canvas.dataset.testid='path';canvas.setAttribute('aria-label','Top-down policy and MPC trajectories');pathWrap.append(canvas);pathCard.body.remove();
  const pathFooter=el('div','ig-pathnote'),pathNote=el('span'),legend=el('div','ig-legend');pathFooter.append(pathNote,legend);pathCard.box.append(pathWrap,pathFooter);main.append(pathCard.box);
  const statsBox=el('div','ig-card ig-stats');main.append(statsBox);statsBox.setAttribute('aria-label','Live telemetry');main.append(pathCard.box);const statNodes=new Map<string,HTMLDivElement>();
  for (const [key,label] of [['rgb','RGB input age'],['odom','Odometry age'],['rtt','Inference RTT'],['clearance','Front clearance'],['vx','Linear proposal'],['wz','Angular proposal'],['steps','Goal distance'],['remaining','Time remaining']]) {
    if (!key || !label) {continue;}const cell=el('div','ig-stat');cell.append(el('label','',label),el('strong','','—'));statNodes.set(key,cell);statsBox.append(cell);
  }
  const taskCard=card('Mission',el('span','ig-meta','Preview or supervised run'));side.append(taskCard.box);
  const modelField=el('div','ig-field'),modelSwitch=el('div','ig-switch'),nomad=button('NoMaD ImageGoal','nomad'),lightnavImageGoal=button('LightNav ImageGoal','lightnav-imagegoal'),lightnav=button('LightNav Navigation','lightnav'),tracking=button('LightNav Tracking','tracking');
  for(const [option,title,description] of [[nomad,'NoMaD','Image goal'],[lightnavImageGoal,'LightNav','Image goal'],[lightnav,'LightNav','Navigation'],[tracking,'LightNav','Tracking']] as const){option.replaceChildren(el('span','ig-optiontitle',title),el('span','ig-optionmeta',description));}
  modelSwitch.setAttribute('role','group');modelSwitch.setAttribute('aria-label','Task selection');
  modelSwitch.append(nomad,lightnavImageGoal,lightnav,tracking);modelField.append(el('label','ig-label','Task'),modelSwitch);taskCard.body.append(modelField);
  const navigationField=el('div','ig-field'),navigationMode=el('select');navigationMode.dataset.testid='navigation-mode';
  for(const [value,label] of [['objnav','ObjNav · Object goal'],['vln','VLN · Route instruction']]){const option=el('option','',label);option.value=value??'';navigationMode.append(option);}
  navigationField.append(el('label','ig-label','Navigation mode'),navigationMode);taskCard.body.append(navigationField);
  const instructionField=el('div','ig-field'),instruction=el('textarea'),instructionLabel=el('label','ig-label');instruction.maxLength=4096;instruction.dataset.testid='instruction';
  instruction.id=`ig-instruction-${globalThis.crypto.randomUUID()}`;instructionLabel.htmlFor=instruction.id;instructionField.append(instructionLabel,instruction);taskCard.body.append(instructionField);
  const imageGoalNote=el('p','ig-subtle','Go2 ImageGoal adapter · experimental reference-image conditioning');imageGoalNote.dataset.testid='imagegoal-adaptation-note';taskCard.body.append(imageGoalNote);
  const trackingState=el('div','ig-tracking-state'),trackingTarget=el('p'),trackingVisibility=el('strong'),trackingNote=el('p','ig-subtle');trackingState.dataset.testid='tracking-state';trackingTarget.dataset.testid='tracking-target';trackingVisibility.dataset.testid='tracking-visibility';
  trackingState.append(trackingTarget,trackingVisibility,trackingNote);taskCard.body.append(trackingState);
  const inputRow=el('div','ig-inputrow'),timeoutField=el('div','ig-field'),timeout=el('input');timeout.type='number';timeout.min='5';timeout.max='3600';timeout.step='5';timeout.value=String(number(initial.timeout_s)??120);timeout.dataset.testid='timeout';
  timeoutField.append(el('label','ig-label','Preview timeout (s)'),timeout);const modeField=el('div','ig-field');modeField.append(el('label','ig-label','Execution'),el('span','ig-fixed','SHADOW'));inputRow.append(timeoutField,modeField);taskCard.body.append(inputRow);
  const start=button('Start preview','start','ig-start'),pause=button('Pause','pause'),resume=button('Resume','resume'),complete=button('Confirm arrival','complete'),stop=button('Stop task','cancel','ig-stop');
  const nativeStart=button('Run NoMaD on robot','native_start','ig-native-start'),nativeStop=button('Stop robot','native_stop','ig-stop'),nativeReason=el('p','ig-subtle'),nativeState=el('div','ig-native-state');
  nativeReason.dataset.testid='native-reason';nativeState.dataset.testid='native-state';nativeState.setAttribute('role','status');
  nativeStart.title='Start a supervised robot run with the frozen goal, up to 60 seconds. Clicking confirms your supervision for this run.';
  taskCard.body.append(start,nativeStart,nativeReason);const controls=el('div','ig-buttonrow');controls.append(pause,resume,complete);taskCard.body.append(controls,stop);badges.append(nativeStop);
  const actionMessage=el('div','ig-actionmsg','Capture a frame or import a JPEG to set an image goal.');actionMessage.dataset.testid='action-status';actionMessage.setAttribute('role','status');taskCard.body.append(actionMessage,el('p','ig-subtle','Arrival is confirmed by the operator. Policy STOP alone does not confirm arrival.'));
  const stateCard=card('Runtime'),taskState=el('div','ig-state','Ready'),taskInfo=el('div','ig-meta','Connect to see the active task'),identity=el('dl','ig-idgrid');
  stateCard.body.append(taskState,taskInfo,nativeState,identity);side.append(stateCard.box);
  const gatesCount=el('span','ig-meta'),gateCard=card('Readiness & sources',gatesCount),gateList=el('ul','ig-gates'),sources=el('div','ig-sources');gateCard.body.append(gateList,sources);side.append(gateCard.box);
  const details=el('details'),summary=el('summary','','Diagnostics & activity'),raw=el('pre'),log=el('pre','ig-log');details.append(summary,log,raw);gateCard.box.append(details);
  const bottom=el('footer','ig-bottom');bottom.append(el('span','',snapshot?'RECORDED SNAPSHOT · READ ONLY':'DEJAVIEW / NATIVE FOXGLOVE'),el('span','','Display and inference are sampled independently. Goals use original inference RGB.'));root.append(bottom);
  const connected=():boolean=>receivedAt>0 && (snapshot || Date.now()-receivedAt<STATUS_STALE_MS) && typeof status.session_id==='string';
  const writable=():boolean=>!snapshot && connected() && typeof context.advertise==='function' && typeof context.publish==='function';
  const canStop=():boolean=>!snapshot && typeof status.session_id==='string' && number(status.epoch)!=undefined && typeof context.advertise==='function' && typeof context.publish==='function';
  function message(text:string,kind=''):void {const readable=displayReason(text);if(readable!==text){lastRuntimeMessage=text;}actionMessage.textContent=readable;actionMessage.className=`ig-actionmsg ${kind}`;}
  function logAction(text:string):void { logRows.unshift(`${new Date().toLocaleTimeString('en-GB',{hour12:false})}  ${text}`);logRows.splice(10);log.textContent=logRows.join('\n'); }
  function rememberInstruction():void {if(taskMode==='tracking'){trackingInstruction=instruction.value;}else if(backend==='lightnav'&&taskMode==='imagegoal'){imageGoalInstruction=instruction.value;}else if(backend==='lightnav'){navigationInstruction=instruction.value;}}
  function persist():void {rememberInstruction();context.saveState({backend,task_mode:taskMode,timeout_s:Number(timeout.value),imagegoal_instruction:imageGoalInstruction,navigation_instruction:navigationInstruction,tracking_instruction:trackingInstruction});}
  function syncModel():void {
    const isTracking=taskMode==='tracking',isImageGoal=backend==='lightnav'&&taskMode==='imagegoal';nomad.classList.toggle('selected',backend==='nomad');lightnavImageGoal.classList.toggle('selected',isImageGoal);lightnav.classList.toggle('selected',backend==='lightnav'&&!isTracking&&!isImageGoal);tracking.classList.toggle('selected',isTracking);for(const option of [nomad,lightnavImageGoal,lightnav,tracking]){option.setAttribute('aria-pressed',String(option.classList.contains('selected')));}instructionField.hidden=backend!=='lightnav';
    instructionLabel.textContent=isTracking?'Target description (required)':isImageGoal?'Additional instruction (optional)':'Navigation instruction (required)';instruction.placeholder=isTracking?'Follow the person in the white T-shirt.':isImageGoal?'Optional guidance for the frozen goal image':'Go to the coat rack beside the windows.';instruction.value=isTracking?trackingInstruction:isImageGoal?imageGoalInstruction:navigationInstruction;
    imageGoalNote.hidden=!isImageGoal;navigationField.hidden=backend!=='lightnav'||isTracking||isImageGoal;navigationMode.value=taskMode==='vln'?'vln':'objnav';
    goal.box.hidden=taskMode!=='imagegoal';pictures.classList.toggle('ig-tracking',taskMode!=='imagegoal');
  }
  function selectMode(nextBackend:string,nextMode:TaskMode):void {rememberInstruction();backend=nextBackend;taskMode=nextMode;syncModel();syncLiveGoal();persist();update();}
  nomad.onclick=()=>{selectMode('nomad','imagegoal');};lightnavImageGoal.onclick=()=>{selectMode('lightnav','imagegoal');};lightnav.onclick=()=>{selectMode('lightnav','objnav');};tracking.onclick=()=>{selectMode('lightnav','tracking');};navigationMode.onchange=()=>{selectMode('lightnav',navigationMode.value==='vln'?'vln':'objnav');};timeout.onchange=persist;instruction.oninput=()=>{persist();update();};syncModel();
  function imageURL(bytes:Uint8Array):string {const url=displayURL(bytes);ownedURLs.add(url);return url;}
  function releaseURL(url:string|undefined):void {
    if(url&&ownedURLs.delete(url)){URL.revokeObjectURL(url);}
  }
  function releaseUnusedURL(url:string|undefined):void {
    if(url&&url!==rgbURL&&url!==goalURL&&camera.img.getAttribute('src')!==url&&goal.img.getAttribute('src')!==url){releaseURL(url);}
  }
  async function prepareImage(image:typeof camera,url:string):Promise<HTMLImageElement> {
    const next=el('img');next.alt=image.img.alt;next.decoding='async';next.dataset.testid=image.img.dataset.testid;next.src=url;
    await next.decode();return next;
  }
  function commitImage(image:typeof camera,next:HTMLImageElement):void {
    image.img.replaceWith(next);image.img=next;image.area.classList.add('has-image');
  }
  function clearImage(which:'rgb'|'goal'):void {
    const image=which==='rgb'?camera:goal,old=image.img.getAttribute('src')??undefined,url=which==='rgb'?rgbURL:goalURL;
    image.img.removeAttribute('src');image.area.classList.remove('has-image');
    if(which==='rgb'){rgbGeneration++;pendingRgb=undefined;rgbURL=undefined;rgbAt=0;rgbSource={kind:'unknown',frameId:''};}
    else{goalURL=undefined;goalAt=0;goalImageReady=false;goalLoadingId='';liveSyncURL='';displayGeneration++;}
    releaseUnusedURL(url);releaseUnusedURL(old);
  }
  function syncLiveGoal():void {
    if(disposed||taskMode!=='imagegoal'||goalSelection(status).mode!=='live'||!rgbURL){return;}
    const url=rgbURL,generation=displayGeneration;
    if(goal.img.getAttribute('src')===url||liveSyncURL===url){return;}
    liveSyncURL=url;
    void prepareImage(goal,url).then(next=>{
      if(disposed||taskMode!=='imagegoal'||generation!==displayGeneration||goalSelection(status).mode!=='live'||rgbURL!==url){return;}
      const old=goal.img.getAttribute('src')??undefined,oldGoal=goalURL;goalURL=undefined;goalImageReady=false;
      commitImage(goal,next);releaseUnusedURL(old);releaseUnusedURL(oldGoal);
    }).catch(()=>{/* Keep the previous good image if decoding fails. */}).finally(()=>{if(liveSyncURL===url){liveSyncURL='';}});
  }
  async function drainRgb():Promise<void> {
    if(rgbLoading){return;}rgbLoading=true;
    try{
      while(!disposed&&pendingRgb){
        const frame=pendingRgb;pendingRgb=undefined;const generation=rgbGeneration,url=imageURL(frame.bytes);
        try{
          const liveGoal=taskMode==='imagegoal'&&goalSelection(status).mode==='live'?prepareImage(goal,url).catch(()=>undefined):Promise.resolve(undefined);
          const [nextCamera,nextGoal]=await Promise.all([prepareImage(camera,url),liveGoal]);
          // eslint-disable-next-line @typescript-eslint/no-unnecessary-condition -- Disposal may occur while image decoding is awaited.
          if(disposed||generation!==rgbGeneration){releaseURL(url);continue;}
          const old=rgbURL;rgbURL=url;rgbAt=frame.receivedAt;rgbSource=frame.source;commitImage(camera,nextCamera);
          if(taskMode==='imagegoal'&&goalSelection(status).mode==='live'&&nextGoal){
            const oldGoal=goalURL,oldSource=goal.img.getAttribute('src')??undefined;goalURL=undefined;goalImageReady=false;
            commitImage(goal,nextGoal);releaseUnusedURL(oldGoal);releaseUnusedURL(oldSource);
          }else{syncLiveGoal();}
          releaseUnusedURL(old);updateCamera();
        }catch{releaseURL(url);/* A failed new frame must not hide the last good frame. */}
      }
    }finally{rgbLoading=false;}
  }
  function showImage(which:'rgb'|'goal',bytes:Uint8Array,selectionId?:string,source?:PreviewSource):void {
    if(which==='rgb'){
      pendingRgb={bytes,source:source??{kind:'unknown',frameId:''},receivedAt:Date.now()};void drainRgb();return;
    }
    if(!selectionId||goalLoadingId===selectionId||(goalURL&&goalImageReady&&goalSelection(status).id===selectionId)){return;}
    const generation=++displayGeneration,url=imageURL(bytes);goalLoadingId=selectionId;
    void prepareImage(goal,url).then(next=>{
      if(disposed||generation!==displayGeneration||goalSelection(status).id!==selectionId){releaseURL(url);return;}
      const old=goalURL,oldSource=goal.img.getAttribute('src')??undefined;goalURL=url;goalAt=Date.now();goalImageReady=true;
      commitImage(goal,next);releaseUnusedURL(old);releaseUnusedURL(oldSource);update();
    }).catch(()=>{
      releaseURL(url);if(disposed||generation!==displayGeneration||goalSelection(status).id!==selectionId){return;}
      message('Goal image could not be decoded. Waiting for a valid matching image.','error');update();
    }).finally(()=>{if(generation===displayGeneration){goalLoadingId='';}});
  }
  async function importGoal(selected:File):Promise<void> {
    const generation=++uploadGeneration;validatingUpload=true;update();
    const obsolete=():boolean=>disposed||generation!==uploadGeneration;
    try{
    const bytes=new Uint8Array(await selected.arrayBuffer());if(obsolete()){return;}
    validateGoalBytes(bytes);
    const bitmap=await createImageBitmap(new Blob([new Uint8Array(bytes).buffer],{type:'image/jpeg'}));const {width,height}=bitmap;bitmap.close();
    if(width>4096||height>2160) {throw new Error('Goal images must be at most 4096 × 2160 pixels.');}if(obsolete()){return;}
    let binary='';for(let i=0;i<bytes.length;i+=32768){binary+=String.fromCharCode(...bytes.subarray(i,i+32768));}
    send('import_goal',{goal:{rgb_jpeg_base64:btoa(binary)}});
    }finally{if(generation===uploadGeneration){validatingUpload=false;update();}}
  }
  upload.onclick=()=> { file.click(); };file.onchange=()=>{const selected=file.files?.[0];if(selected){void importGoal(selected).catch((error:unknown)=> { message(String(error),'error'); });}file.value='';};
  function send(action:Action,payload:JsonMap={}):void {
    const isStop=stopAction(action);
    if(!(action==='native_stop'?canStop():writable())){message('Runtime status is stale or read-only. Restore the Foxglove connection.','error');return;}
    if(isStop&&requests.stopping){return;}
    if(!isStop&&(uncertainty||waitingEpoch!=undefined)){message('The previous action was not acknowledged. Stop the task before sending another action.','error');return;}
    let sentId:string|undefined;
    try {
      const id=globalThis.crypto.randomUUID();sentId=id;const value=command(action,status,payload,id);requests.begin(id,action,Date.now());
      if(isStop){uploadGeneration++;validatingUpload=false;}
      if(!advertised){context.advertise?.(TOPICS.command,'std_msgs/msg/String',{datatypes:new Map([['std_msgs/msg/String',STRING_SCHEMA],['std_msgs/String',STRING_SCHEMA]])});advertised=true;}
      context.publish?.(TOPICS.command,{data:JSON.stringify(value)});message(isStop?'Stop requested. Waiting for runtime acknowledgement…':`${ACTION_NAMES[action]} requested. Waiting for acknowledgement…`);logAction(`${ACTION_NAMES[action]} · sent`);update();
    } catch(error) {if(sentId){requests.settle(sentId);}message(error instanceof Error?error.message:String(error),'error');advertised=false;update();}
  }
  function startPayload():JsonMap {return backend==='lightnav'?{backend:'lightnav',task_mode:taskMode,instruction:instruction.value,...(taskMode==='imagegoal'?{selection_id:goalSelection(status).id}:{})}:{backend,selection_id:goalSelection(status).id,instruction:''};}
  start.onclick=()=> { send('start',{...startPayload(),execution_mode:'shadow',timeout_s:Number(timeout.value)}); };
  nativeStart.onclick=()=>{send('native_start',startPayload());};
  nativeStop.onclick=()=>{send('native_stop');};
  capture.onclick=()=> { send('capture_goal'); };clear.onclick=()=> { send('clear_goal'); };
  pause.onclick=()=> { send('pause',{reason:'operator_pause'}); };resume.onclick=()=> { send('resume',{reason:'operator_resume'}); };stop.onclick=()=> { send(record(status.native_executor).running===true||record(status.task).execution_mode==='supervised_native'||[...requests.pending.values()].some(value=>value.action==='native_start')?'native_stop':'cancel',{reason:'operator_stop'}); };complete.onclick=()=> { send('complete',{evidence:'operator_confirmed'}); };
  function acknowledge(result:JsonMap):void {
    const pending=requests.settle(str(result.request_id,''));if(!pending){return;}
    if(pending.superseded){logAction(`${ACTION_NAMES[pending.action]} · superseded by Stop`);return;}
    if(result.ok!==true){const original=str(result.error,'Runtime rejected the action');lastRuntimeMessage=original;const error=displayReason(original);message(error,'error');logAction(`${ACTION_NAMES[pending.action]} · ${error}`);update();return;}
    const body=record(result.result),ackStatus=typeof body.session_id==='string'?body:record(body.status);
    if((pending.action==='native_start'||pending.action==='native_stop')&&typeof ackStatus.session_id!=='string'){
      if(Object.keys(record(body.executor)).length>0){status={...status,native_executor:record(body.executor)};}
      if(pending.action==='native_start'){waitingEpoch=number(status.epoch);message('Run accepted. Waiting for executor status; motion is not confirmed.');}
      else{uncertainty=false;waitingEpoch=undefined;message('Stop accepted. Waiting for executor exit.');}
      logAction(`${ACTION_NAMES[pending.action]} · accepted`);update();return;
    }
    if(ackStatus.session_id!==status.session_id||number(ackStatus.epoch)==undefined){
      uncertainty=true;message('Acknowledgement is missing the current session state. Check the connection and stop the task.','error');update();return;
    }
    const applied=applyStatus(ackStatus);waitingEpoch=undefined;
    if(stopAction(pending.action)){uncertainty=false;}
    if(pending.action==='capture_goal'||pending.action==='import_goal'){
      const selection=goalSelection(status),ackSelection=goalSelection(ackStatus),jpeg=record(body.goal).rgb_jpeg_base64;
      if(!selection.ready||selection.id!==ackSelection.id){logAction(`${ACTION_NAMES[pending.action]} · obsolete goal acknowledgement ignored`);update();return;}
      if(typeof jpeg!=='string'){message('Goal frozen. Waiting for its matching image.');update();return;}
      try{const bytes=Uint8Array.from(atob(jpeg),c=>c.charCodeAt(0));validateGoalBytes(bytes);showImage('goal',bytes,selection.id);}catch{message('The frozen-goal response contains invalid JPEG data. Waiting for a matching image.','error');update();return;}
    }else if(!applied&&number(ackStatus.epoch)!==number(status.epoch)){
      logAction(`${ACTION_NAMES[pending.action]} · obsolete acknowledgement ignored`);update();return;
    }
    message(pending.action==='capture_goal'||pending.action==='import_goal'?'Goal confirmed by runtime. Ready to start.':pending.action==='clear_goal'?'Live preview restored. Capture or import a new goal.':pending.action==='native_start'?'Robot run confirmed by runtime. Check authorization and executor status below.':`${ACTION_NAMES[pending.action]} confirmed by runtime.`,'good');
    logAction(`${ACTION_NAMES[pending.action]} · acknowledged`);update();
  }
  function applyStatus(next:JsonMap):boolean {
    if(typeof next.session_id!=='string'||number(next.epoch)==undefined){return false;}
    if(next.session_id===status.session_id&&number(next.epoch)!=undefined&&number(status.epoch)!=undefined){
      if((next.epoch as number)<(status.epoch as number)){return false;}
      if(next.epoch===status.epoch&&number(next.timestamp_ns)!=undefined&&number(status.timestamp_ns)!=undefined&&(next.timestamp_ns as number)<(status.timestamp_ns as number)){return false;}
    }
    const changedSession=status.session_id!=undefined&&status.session_id!==next.session_id;
    const changedCamera=status.session_id!=undefined&&cameraIdentity(status)!==cameraIdentity(next);
    if(changedSession||changedCamera){requests.reset();uploadGeneration++;validatingUpload=false;uncertainty=false;waitingEpoch=undefined;advertised=false;clearImage('rgb');clearImage('goal');logAction(changedSession?'Runtime session changed':'Camera source changed');}
    const selection=goalSelection(next),key=`${next.session_id}/${selection.mode}/${selection.id}`;
    if(key!==goalIdentity){goalIdentity=key;clearImage('goal');}
    if(waitingEpoch!=undefined&&next.epoch!==waitingEpoch){waitingEpoch=undefined;}status=next;receivedAt=Date.now();
    if(selection.mode==='live'){syncLiveGoal();}return true;
  }
  function updateCamera(now=Date.now()):void {
    camera.meta.textContent=snapshot?'Recorded image':rgbAt>0?`Received ${metric((now-rgbAt)/1000,' s',1)} ago`:'Waiting for image';
    cameraFoot.textContent=previewTimingLabel(rgbSource,snapshot?rgbAt:now);
    cameraFoot.title=`* ${rgbSource.kind==='rpc_receive'?'Unitree provides host RPC receipt time, not sensor exposure time. This does not measure exposure latency.':'Browser clock minus preview source time. Machine clock offsets affect this value; it is not a standalone network latency measure.'}${rgbSource.frameId?` Camera frame: ${rgbSource.frameId}`:''}`;
  }
  details.ontoggle=()=>{if(details.open){raw.textContent=JSON.stringify({status,bridge,lastRuntimeMessage},null,2);}};
  function syncSubscriptions():void {
    // subscribe() replaces this panel's complete subscription set. Keep the
    // live channels and fetch a frozen original JPEG only until it has decoded.
    const wantGoal=taskMode==='imagegoal'&&goalSelection(status).ready&&!goalImageReady;
    if(subscribedGoal===wantGoal){return;}
    subscribedGoal=wantGoal;
    context.subscribe([{topic:TOPICS.status},{topic:TOPICS.bridge},{topic:TOPICS.result},{topic:TOPICS.rgb},...(wantGoal?[{topic:TOPICS.goal}]:[])]);
  }
  function update():void {
    if(disposed){return;}syncSubscriptions();const now=Date.now(),elapsed=snapshot?0:receivedAt>0?now-receivedAt:0,isConnected=connected(),task=record(status.task),sensors=record(status.sensors),isActive=active(status);
    const locked=!writable()||requests.busy||requests.stopping||validatingUpload||uncertainty||waitingEpoch!=undefined,selection=goalSelection(status),executor=record(status.native_executor),native=record(status.native_execution),nativeTask=task.execution_mode==='supervised_native';
    connection.textContent=snapshot?'● Snapshot':isConnected?'● Connected':receivedAt>0?'● Status stale':'● Connecting';connection.className=`ig-pill ${snapshot?'warn':isConnected?'online':receivedAt>0?'error':''}`;
    stateBadge.textContent=str(status.state,'IDLE');stateBadge.className=`ig-pill ${isActive?'online':''}`;
    const isTracking=taskMode==='tracking',isNavigation=taskMode==='objnav'||taskMode==='vln',actualTracking=task.task_mode==='tracking',actualNavigation=task.task_mode==='objnav'||task.task_mode==='vln',trackingStatus=record(status.tracking);
    taskState.textContent=status.state==='SUCCEEDED'&&['go2_imagegoal_model_stop','official_lightnav_model_stop'].includes(str(record(task.outcome).reason))?'Policy stopped · arrival unconfirmed':STATE_NAMES[str(status.state,'IDLE')]??str(status.state,'Ready');taskInfo.textContent=typeof task.backend==='string'?`${str(task.backend).toUpperCase()} · ${actualTracking?'Tracking':actualNavigation?str(task.task_mode).toUpperCase():'ImageGoal'} · ${str(task.execution_mode,'shadow')}${actualTracking||actualNavigation?'':` · goal v${String(number(task.goal_version)??'—')}`}`:'Choose an image goal, navigation instruction, or tracking target.';
    const synthetic=status.synthetic_feedback===true||status.synthetic===true;
    const selectedModel=backend==='lightnav'?'LightNav':'NoMaD',taskModel=task.backend==='lightnav'?'LightNav':task.backend==='nomad'?'NoMaD':str(task.backend,'Unknown policy');
    nativeStart.textContent=isTracking?'Run LightNav tracking':`Run ${selectedModel} on robot`;nativeStart.title=isNavigation?'Run official navigation with RGB history and the required instruction, up to 60 seconds. No goal image is used. Clicking confirms supervision.':isTracking?'Track the described target using RGB observations, up to 60 seconds. No goal image is used. Clicking confirms supervision.':`Run ${selectedModel} with the full frozen goal image, up to 60 seconds. Clicking confirms supervision.`;
    start.textContent=isTracking?'Start tracking preview':'Start preview';
    bannerMode.textContent=nativeTask?`NATIVE · ${taskModel}${actualTracking?' tracking':' robot run'}`:`SHADOW · ${actualTracking?'TRACKING':'PREVIEW'}`;
    banner.dataset.mode=snapshot?'snapshot':synthetic?'simulation':nativeTask?'native':'shadow';
    bannerText.textContent=snapshot?'Recorded snapshot. All controls are disabled; this is not a live connection.':synthetic?'Simulated feedback. Trajectories are previews, not evidence of robot navigation.':nativeTask?`Active ${taskModel}${actualTracking?' tracking':' robot run'}. Check authorization and executor status to confirm execution.`:isTracking?'Track a target from RGB history and its description. Robot runs require supervision and end after 60 seconds.':isNavigation?'Official LightNav navigation uses RGB history and language. Policy STOP ends the task.':'Preview observes the policy. Run on robot uses the full frozen goal for up to 60 seconds under supervision.';
    const instructionBlocked=isTracking?trackingStartReason(status,backend,instruction.value):isNavigation?navigationStartReason(status,backend,instruction.value,taskMode):imageGoalStartReason(status,backend,instruction.value),goalBlocked=taskMode==='imagegoal'&&(!selection.ready||!goalImageReady);
    start.disabled=locked||executor.running===true||!!instructionBlocked||goalBlocked;pause.disabled=locked||!isActive||status.state==='PAUSED';resume.disabled=locked||!isActive||status.state!=='PAUSED'||task.model_stop_latched===true;complete.disabled=locked||!isActive;complete.hidden=isTracking||actualTracking;
    const nativeBlocked=nativeStartReason(status,backend,taskMode)||instructionBlocked;nativeStart.disabled=locked||!!nativeBlocked||goalBlocked;nativeReason.textContent=nativeBlocked||(goalBlocked?'Waiting for the matching frozen goal image.':isTracking?'Supervised · described target · 60 s limit · remote takeover available':isNavigation?'Supervised · RGB + language · 60 s limit · remote takeover available':'Supervised · frozen full image · 60 s limit · remote takeover available');
    trackingState.hidden=!isTracking&&!actualTracking;
    trackingTarget.textContent=`${actualTracking&&isActive?'Active target':'Selected target'}: ${actualTracking&&isActive?str(trackingStatus.instruction,str(task.instruction,'')):instruction.value.trim()||'Enter a target description'}`;
    const targetState=str(trackingStatus.target_state,'unknown'),visibility=actualTracking&&isActive?(targetState==='holding'?'Motion held · observing':trackingStatus.visible===true?'Target visible':trackingStatus.visible===false?'Target not visible · observing':'Visibility unknown'):'Tracking not started';
    trackingVisibility.textContent=`${actualTracking&&(!isConnected||!isActive)?'Historical · ':''}${visibility}`;
    trackingNote.textContent=actualTracking&&isActive?`Policy STOP or a zero path holds motion while observation continues. Visibility is diagnostic${str(trackingStatus.hold_reason,'')?` · ${displayReason(str(trackingStatus.hold_reason))}`:''}.`:'Describe the target appearance. Selecting a mode does not change the running task.';
    const nativeCommand=record(native.command);nativeState.textContent=`${nativeTask?`${taskModel} `:''}Executor: ${executor.running===true?'process running':str(executor.state,'not started')} · motion authorization: ${native.authorized===true?'granted':'not granted'}${Object.keys(nativeCommand).length>0?` · vx ${metric(nativeCommand.vx_m_s,' m/s',2)} / wz ${metric(nativeCommand.wz_rad_s,' rad/s',2)}`:''}${str(executor.error,'')!==''?` · ${displayReason(str(executor.error))}`:''}`;
    nativeState.classList.toggle('ig-attention',executor.running===true||native.authorized===true);nativeStop.disabled=!canStop()||requests.stopping;
    upload.disabled=locked||taskMode!=='imagegoal';clear.disabled=locked||taskMode!=='imagegoal'||!selection.ready;stop.disabled=!writable()||requests.stopping;capture.disabled=locked||taskMode!=='imagegoal'||selection.ready||(sourceAgeMs(sensors.rgb,elapsed)??Infinity)>1000;nomad.disabled=lightnavImageGoal.disabled=lightnav.disabled=tracking.disabled=snapshot||requests.busy||requests.stopping;timeout.disabled=instruction.disabled=navigationMode.disabled=snapshot;
    goal.box.querySelector('h2')!.textContent=selection.ready?'Goal image · frozen':'Goal image · live preview';goalMode.textContent=selection.ready?'Frozen · ready for task':'Live preview · no goal captured';goalMode.className=`ig-imagestate ${selection.ready?'frozen':''}`;
    goalFoot.textContent=selection.ready?`${selection.source==='import'?'Imported JPEG':'Camera capture'} · ${selection.id.slice(0,8)}`:'Capture or import a goal to begin';goalFoot.className=selection.ready?'ig-goalbadge':'';
    goal.meta.textContent=selection.ready?(goalImageReady&&goalAt>0?'Frozen image ready':'Waiting for frozen image'):'Mirrors live camera';
    const rgbAge=sourceAgeMs(sensors.rgb,elapsed),odomAge=sourceAgeMs(sensors.odom,elapsed),depthAge=sourceAgeMs(sensors.depth,elapsed);
    updateCamera(now);
    const v=record(status.proposal),m=record(status.metrics),pv=record(status.preview),plan=record(status.plan);
    const displayStats:Record<string,[string,boolean]>={rgb:[metric(rgbAge,' ms',0),(rgbAge??Infinity)>300],odom:[metric(odomAge,' ms',0),(odomAge??Infinity)>300],rtt:[metric(pv.round_trip_ms??plan.round_trip_ms??m.last_inference_ms??m.last_rtt_ms,' ms',0),false],clearance:[metric(record(sensors.depth).clearance_m,' m',2),false],vx:[metric(v.vx_m_s,' m/s',3),false],wz:[metric(v.wz_rad_s,' rad/s',3),false],steps:[metric(pv.distance_steps,' steps',2),false],remaining:[metric(task.remaining_s,' s',0),false]};
    for(const [key,[value,warn]] of Object.entries(displayStats)){const cell=statNodes.get(key);if(cell){cell.querySelector('strong')!.textContent=value;cell.classList.toggle('warn',warn);}}
    identity.replaceChildren();for(const [key,value] of Object.entries({Session:str(status.session_id),Task:str(task.task_id),Goal:str(task.goal_id),Version:`${String(number(task.goal_version)??'—')} / epoch ${String(number(status.epoch)??'—')}`})){identity.append(el('dt','',key),el('dd','',value));}
    const checks=gates(status);gateList.replaceChildren();gatesCount.textContent=checks.length>0?`${checks.length} gates`:'Ready';
    if(checks.length===0){gateList.append(el('li','good',isConnected?'No runtime gates active':'Waiting for runtime status'));}else {for(const check of checks.slice(0,8)){gateList.append(el('li','',displayReason(check)));}}
    if(checks.length>8){gateList.append(el('li','',`${checks.length-8} more in diagnostics`));}
    sources.replaceChildren();const sourceRows:[string,string,boolean][]=[['Runtime',snapshot?'Recorded, not live':receivedAt>0?`${metric(elapsed,' ms',0)} ago`:'Not received',!isConnected],['ROS bridge',snapshot?'Snapshot, not live':bridgeAt>0?`${metric((now-bridgeAt)/1000,' s')} ago`:'Not received',bridgeAt===0||now-bridgeAt>3000],['RGB / depth age',`${metric(rgbAge,' ms',0)} / ${metric(depthAge,' ms',0)}`,(depthAge??Infinity)>300],['Source type',synthetic?'Simulation / replay':'Live source (runtime-declared)',synthetic]];for(const [label,value,warn] of sourceRows){
      const row=el('div','ig-source');row.append(el('span','',label),el('span',warn?'warn':'',value));sources.append(row);
    }
    if(details.open){raw.textContent=JSON.stringify({status,bridge,lastRuntimeMessage},null,2);}
    const view=pathView(status);pathLabel.textContent=view.frame;pathLabel.className=`ig-meta ${view.historical?'ig-attention':''}`;
    pathNote.textContent=`${view.historical?'Historical output · ':''}${view.note}${view.sourceAgeS==undefined?'':` · source age ${metric(view.sourceAgeS+elapsed/1000,' s')}`}`;
    legend.replaceChildren();for(const line of view.lines){const item=el('span'),swatch=el('i');swatch.style.background=line.color;item.append(swatch,document.createTextNode(line.label));legend.append(item);}drawPath(canvas,view,{disconnected:!isConnected});
  }
  syncSubscriptions();context.watch('currentFrame');context.watch('didSeek');context.watch('topics');context.watch('colorScheme');
  context.onRender=(renderState,done)=>{
    try{
      let dashboardChanged=false;
      if(renderState.colorScheme&&root.dataset.theme!==renderState.colorScheme){root.dataset.theme=renderState.colorScheme;dashboardChanged=true;}
      if(renderState.didSeek===true){requests.reset();uploadGeneration++;validatingUpload=false;status={};receivedAt=0;clearImage('rgb');clearImage('goal');dashboardChanged=true;}
      if(renderState.topics?.length===0){requests.reset();uploadGeneration++;validatingUpload=false;status={};receivedAt=0;advertised=false;clearImage('rgb');clearImage('goal');dashboardChanged=true;}
      const frame=renderState.currentFrame??[];
      for(const event of frame){if(event.topic===TOPICS.status){const value=parseString(event.message);if(value){applyStatus(value);}else {message('Received malformed runtime status.','error');}dashboardChanged=true;}}
      let latestRgb:unknown;
      for(const event of frame){
        if(event.topic===TOPICS.bridge){const value=parseString(event.message);if(value){bridge=value;bridgeAt=Date.now();dashboardChanged=true;}}
        else if(event.topic===TOPICS.result){const value=parseString(event.message);if(value){acknowledge(value);}dashboardChanged=true;}
        else if(event.topic===cameraTopic(status)){
          const expectedFrame=str(record(record(status.sensors).rgb).frame_name,''),actualFrame=previewSource(event.message).frameId;
          if(!expectedFrame||actualFrame===expectedFrame||actualFrame.includes('display_time_preview')){latestRgb=event.message;}
        }
        else if(event.topic===TOPICS.goal&&taskMode==='imagegoal'&&!goalURL&&goalLoadingId!==goalSelection(status).id&&matchesGoalFrame(event.message,goalSelection(status))){const bytes=compressedBytes(event.message);if(bytes){showImage('goal',bytes,goalSelection(status).id);dashboardChanged=true;}}
      }
      // Foxglove can deliver multiple camera messages in one render frame. Only
      // decode the newest one; RGB-only frames do not rebuild the whole dashboard.
      if(latestRgb!=undefined){const bytes=compressedBytes(latestRgb);if(bytes){showImage('rgb',bytes,undefined,previewSource(latestRgb));}}
      if(dashboardChanged){update();}else {updateCamera();}
    }catch(error){message(error instanceof Error?error.message:String(error),'error');}finally{done();}
  };
  const timer=setInterval(()=>{
    for(const item of requests.expired(Date.now())){requests.settle(item.id);if(item.generation!==requests.generation){continue;}uncertainty=true;message(`${ACTION_NAMES[item.action]} was not acknowledged. Outcome unknown; check the connection and stop the task.`,'error');logAction(`${ACTION_NAMES[item.action]} · acknowledgement timed out`);}
    update();
  },350);
  const resize=new ResizeObserver(()=> { update(); });resize.observe(pathWrap);update();
  return ()=>{disposed=true;uploadGeneration++;clearInterval(timer);resize.disconnect();context.onRender=undefined;context.unsubscribeAll();if(advertised){context.unadvertise?.(TOPICS.command);}clearImage('rgb');clearImage('goal');for(const url of ownedURLs){releaseURL(url);}root.remove();};
}

export function drawPath(canvas:HTMLCanvasElement,view:PathView,options:{disconnected?:boolean}={}):void {
  const disconnected=options.disconnected===true;
  const rect=canvas.getBoundingClientRect(),width=Math.max(260,rect.width),height=Math.max(180,rect.height),dpr=Math.min(2,globalThis.devicePixelRatio>0?globalThis.devicePixelRatio:1);
  canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);const ctx=canvas.getContext('2d');if(!ctx){return;}ctx.scale(dpr,dpr);ctx.clearRect(0,0,width,height);
  const palette=globalThis.getComputedStyle(canvas),pathText=palette.getPropertyValue('--path-text').trim()||'#9daebb',pathGrid=palette.getPropertyValue('--path-grid').trim()||'#28313b',poseColor=palette.getPropertyValue('--pose').trim()||'#eef1f4',pathBackground=palette.getPropertyValue('--path-bg').trim()||'#13181e';
  if(view.mode==='empty'){ctx.fillStyle=pathText;ctx.textAlign='center';ctx.font='12px sans-serif';ctx.fillText(view.note,width/2,height/2,width-32);return;}
  const all=view.lines.flatMap(line=>line.points);if(view.pose){all.push(view.pose);}if(all.length===0){return;}
  let xmin=Math.min(...all.map(p=>p[0])),xmax=Math.max(...all.map(p=>p[0])),ymin=Math.min(...all.map(p=>p[1])),ymax=Math.max(...all.map(p=>p[1]));if(xmax-xmin<.5){xmin-=.25;xmax+=.25;}if(ymax-ymin<.5){ymin-=.25;ymax+=.25;}
  const pad=.18;xmin-=pad;xmax+=pad;ymin-=pad;ymax+=pad;const scale=Math.min((width-90)/(ymax-ymin),(height-55)/(xmax-xmin)),cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;
  const xy=(p:Point):[number,number]=>[width/2-(p[1]-cy)*scale,height/2-(p[0]-cx)*scale];
  const range=Math.max(xmax-xmin,ymax-ymin);const grid=range<2?.2:range<5?.5:range<12?1:5;ctx.strokeStyle=pathGrid;ctx.lineWidth=1;
  for(let x=Math.ceil(xmin/grid)*grid,count=0;x<=xmax&&count<100;x+=grid,count++){const a=xy([x,ymin]),b=xy([x,ymax]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
  for(let y=Math.ceil(ymin/grid)*grid,count=0;y<=ymax&&count<100;y+=grid,count++){const a=xy([xmin,y]),b=xy([xmax,y]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
  ctx.globalAlpha=(disconnected||view.historical)?0.48:1;
  for(const line of view.lines){if(line.points.length===0){continue;}const color=line.color.startsWith('var(')?palette.getPropertyValue(line.color.slice(4,-1)).trim():line.color;ctx.strokeStyle=color;ctx.lineWidth=line.label==='MPC prediction'?2.5:3;ctx.lineJoin='round';ctx.lineCap='round';ctx.beginPath();line.points.forEach((p,index)=>{const q=xy(p);if(index===0){ctx.moveTo(...q);}else {ctx.lineTo(...q);}});ctx.stroke();const last=line.points[line.points.length-1];if(last){ctx.fillStyle=color;ctx.beginPath();ctx.arc(...xy(last),3.5,0,Math.PI*2);ctx.fill();}}
  const pose=view.pose;if(pose){const p=xy(pose),yaw=pose[2]??0;ctx.save();ctx.translate(...p);ctx.rotate(-yaw);ctx.fillStyle=poseColor;ctx.strokeStyle=pathBackground;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(0,-9);ctx.lineTo(-6,7);ctx.lineTo(0,4);ctx.lineTo(6,7);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();}
  ctx.globalAlpha=1;ctx.fillStyle=pathText;ctx.font='10px sans-serif';ctx.textAlign='left';ctx.fillText(`Grid ${grid} m  ·  ↑ +x  ← +y`,13,height-12);ctx.textAlign='right';ctx.fillStyle=pathText;ctx.fillText(view.mode==='body'?'LOCAL / UNANCHORED':'ODOM',width-13,height-12);
  if(disconnected||view.historical){ctx.textAlign='left';ctx.fillStyle='#e1b76c';ctx.fillText(disconnected?'Stale status · historical trajectory only':'Historical output · check source freshness',13,20);}
}
