// Exercise the shipped panel callbacks without ROS, HTTP, SDK or a browser.
const module={exports:{}};
const timers=[];
globalThis.setInterval=fn=>{timers.push(fn);return timers.length;};
globalThis.clearInterval=()=>{};
globalThis.ResizeObserver=class {observe(){}disconnect(){}};
globalThis.Blob=class {};
globalThis.URL={createObjectURL:()=>`blob:${Math.random()}`,revokeObjectURL:()=>{}};
let nextId=0;globalThis.crypto={randomUUID:()=>`test-request-${++nextId}`};
class TestNode {
  constructor(tag='div'){
    this.tagName=tag;this.children=[];this.dataset={};this.style={};this.attributes={};this.textContent='';this.className='';this.disabled=false;this.hidden=false;this.open=false;
    this.classList={add:()=>{},remove:()=>{},toggle:()=>{},contains:()=>false};
  }
  append(...nodes){this.children.push(...nodes);for(const node of nodes){if(node&&typeof node==='object')node.parent=this;}}
  remove(){if(this.parent)this.parent.children=this.parent.children.filter(node=>node!==this);}
  replaceWith(next){if(this.parent){const p=this.parent;p.children[p.children.indexOf(this)]=next;next.parent=p;}}
  replaceChildren(...nodes){this.children=[];this.append(...nodes);}
  setAttribute(name,value){this.attributes[name]=value;}
  getAttribute(name){return name==='src'?this.src??null:this.attributes[name]??null;}
  removeAttribute(name){if(name==='src')delete this.src;else delete this.attributes[name];}
  querySelector(tag){for(const n of this.children){if(n?.tagName===tag)return n;const child=n?.querySelector?.(tag);if(child)return child;}return null;}
  getBoundingClientRect(){return {width:600,height:300};}
  getContext(){return null;}
  decode(){return Promise.resolve();}
}
globalThis.document={createElement:tag=>new TestNode(tag),createTextNode:text=>text};
function byId(parent,id){if(parent.dataset?.testid===id)return parent;for(const n of parent.children??[]){const child=byId(n,id);if(child)return child;}return undefined;}
async function flush(){for(let i=0;i<8;i++)await Promise.resolve();}
async function panelChecks(exports){
  const published=[],panelElement=new TestNode(),context={panelElement,initialState:{},setDefaultPanelTitle:()=>{},saveState:()=>{},subscribe:()=>{},watch:()=>{},advertise:()=>{},publish:(topic,message)=>published.push(JSON.parse(message.data)),unsubscribeAll:()=>{},unadvertise:()=>{}};
  let panel;
  exports.activate({registerPanel:value=>{panel=value;}});assert.equal(panel.name,'ImageGoal');const dispose=panel.initPanel(context);
  const get=id=>byId(panelElement,id),emitStatus=s=>context.onRender({currentFrame:[{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]},()=>{});
  let status={session_id:'panel-test-session',epoch:1,timestamp_ns:Date.now()*1e6,state:'IDLE',profile:'external',goal_selection:{mode:'frozen',ready:true,selection_id:'chosen'},sensors:{rgb:{age_s:.01}},execution_capability:{supervised_native_configured:false,operator_supervised:false},native_executor:{state:'idle',running:false},native_execution:{authorized:false}};
  emitStatus(status);assert.equal(get('native_start').disabled,true);assert.match(get('native-reason').textContent,/[Ss]upervised-native/);
  status={...status,execution_capability:{supervised_native_configured:true,operator_supervised:true,supervised_native_backends:['nomad','lightnav'],task_modes:['imagegoal','objnav','vln','tracking'],supervised_native_task_modes:['imagegoal','objnav','vln','tracking']}};emitStatus(status);
  context.onRender({currentFrame:[{topic:'/imagegoal/shadow/goal/compressed',message:{header:{frame_id:'imagegoal_frozen_goal_chosen'},data:new Uint8Array([255,216,255])}}]},()=>{});await flush();
  assert.equal(get('native_start').disabled,false);
  get('native_start').onclick();assert.equal(published.at(-1).action,'native_start');assert.deepEqual(published.at(-1).payload,{expected_session_id:'panel-test-session',expected_epoch:1,selection_id:'chosen',backend:'nomad',instruction:'',timeout_s:60,operator_present:true});assert.equal(get('native_start').disabled,true);
  assert.equal(get('native_stop').disabled,false);get('native_stop').onclick();assert.equal(published.at(-1).action,'native_stop');assert.equal(published.at(-1).payload.expected_session_id,'panel-test-session');
  const start=published.at(-2),stop=published.at(-1);
  const stopped={...status,epoch:3,timestamp_ns:Date.now()*1e6,state:'CANCELLED',native_executor:{state:'stopped',running:false}};
  context.onRender({currentFrame:[{topic:'/imagegoal/operator/result',message:{data:JSON.stringify({request_id:stop.request_id,ok:true,result:stopped})}}]},()=>{});
  assert.equal(get('native_start').disabled,false);
  context.onRender({currentFrame:[{topic:'/imagegoal/operator/result',message:{data:JSON.stringify({request_id:start.request_id,ok:true,result:{...status,epoch:2,state:'TRACKING',native_executor:{running:true}}})}}]},()=>{});
  assert.match(get('native-state').textContent,/stopped/);assert.equal(get('native_start').disabled,false);
  get('lightnav').onclick();assert.equal(get('native_start').disabled,true);get('instruction').value='Go to the coat rack.';get('instruction').oninput();assert.equal(get('native_start').disabled,false);assert.match(get('native_start').textContent,/Run LightNav on robot/);get('nomad').onclick();
  status={...stopped,epoch:4,timestamp_ns:Date.now()*1e6,task:{task_id:'native-task',execution_mode:'supervised_native',backend:'nomad'},native_executor:{running:true,state:'running'},native_execution:{authorized:false}};emitStatus(status);
  assert.match(get('native-state').textContent,/process running.*not granted/);assert.equal(get('native_start').disabled,true);
  assert.equal(get('native_stop').disabled,false);get('cancel').onclick();assert.equal(published.at(-1).action,'native_stop');
  dispose();
  print('Native panel capability, fixed identity, pending start/stop, late ACK, LightNav and executor status checks passed.');
}
function runPanelTests(exports){panelChecks(exports).catch(error=>{printerr(error.stack??String(error));imports.system.exit(1);});}
