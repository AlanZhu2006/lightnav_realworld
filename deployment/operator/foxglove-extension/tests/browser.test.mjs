import test from 'node:test';
import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';
const directory=join(dirname(fileURLToPath(import.meta.url)),'..','demo');
const webDirectory=join(directory,'..','..','web');
let server,browser,base;
test.before(async()=>{
  server=createServer(async(req,res)=>{try{const path=new URL(req.url,'http://localhost').pathname;const file=path==='/'?'index.html':path.slice(1);if(file.includes('..'))throw Error();const isWeb=file.startsWith('web/')||['imagegoal.js','imagegoal.css','lightnav-base.css'].includes(file);const body=await readFile(join(isWeb?webDirectory:directory,file.replace(/^web\//,'')));res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.jpg')?'image/jpeg':'text/html');res.end(body);}catch{res.statusCode=404;res.end();}});
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));base=`http://127.0.0.1:${server.address().port}`;
  browser=await chromium.launch({headless:true,...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{}),args:['--no-sandbox']});
});
test.after(async()=>{await browser?.close();await new Promise(resolve=>server?.close(resolve));});
test('native panel renders body-local preview; capture/start/pause/resume remain shadow commands',async()=>{
  const page=await browser.newPage({viewport:{width:1320,height:1000}});const errors=[];page.on('pageerror',e=>errors.push(e.message));await page.goto(base);await page.getByTestId('connection').filter({hasText:'Connected'}).waitFor();
  assert.match(await page.locator('.ig-pathnote').innerText(),/at image capture/);assert.match(await page.locator('.ig-cardhead').allTextContents().then(x=>x.join(' ')),/unanchored/);
  assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('rgb').getAttribute('src'),await page.getByTestId('goal').getAttribute('src'));assert.match(await page.getByTestId('goal-mode').innerText(),/no goal captured/);
  await page.getByTestId('capture').click();assert.equal(await page.getByTestId('start').isDisabled(),true);await page.waitForFunction(()=>document.querySelector('[data-testid="start"]').disabled===false);
  await page.getByTestId('start').click();await page.waitForFunction(()=>document.querySelector('[data-testid="action-status"]').textContent.includes('confirmed by runtime'));
  let commands=await page.evaluate(()=>globalThis.imagegoalDemo.published);const start=commands.find(c=>c.action==='start');assert.equal(start.payload.execution_mode,'shadow');assert.equal(start.payload.expected_epoch,8);assert.equal(start.payload.selection_id,'selection-1');assert.equal(start.payload.goal,undefined);assert.match(await page.getByTestId('goal-mode').innerText(),/Frozen/);
  await page.getByTestId('pause').click();await page.waitForFunction(()=>document.querySelector('[data-testid="resume"]').disabled===false);await page.getByTestId('resume').click();
  await page.waitForFunction(()=>globalThis.imagegoalDemo.published.some(c=>c.action==='resume'));commands=await page.evaluate(()=>globalThis.imagegoalDemo.published);assert.equal(commands.find(c=>c.action==='pause').payload.expected_epoch,9);assert.equal(commands.find(c=>c.action==='resume').payload.expected_epoch,10);
  await page.screenshot({path:join(directory,'preview-body.png'),fullPage:true});assert.deepEqual(errors,[]);await page.close();
});
test('stop remains enabled while start awaits its acknowledgement',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.getByTestId('capture').click();await page.waitForFunction(()=>document.querySelector('[data-testid="start"]').disabled===false);
  await page.evaluate(()=>{document.querySelector('[data-testid="start"]').click();if(document.querySelector('[data-testid="cancel"]').disabled)throw Error('Stop disabled while request pending');document.querySelector('[data-testid="cancel"]').click();});
  await page.waitForFunction(()=>document.querySelector('[data-testid="action-status"]').textContent.includes('Stop confirmed by runtime'));
  const commands=await page.evaluate(()=>globalThis.imagegoalDemo.published);assert.deepEqual(commands.map(c=>c.action),['capture_goal','start','cancel']);await page.waitForTimeout(250);assert.equal(await page.locator('.ig-state').innerText(),'Stopped');await page.close();
});
test('stale source disables task controls and visibly marks the connection',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=stale');await page.waitForFunction(()=>document.querySelector('[data-testid="connection"]').textContent.includes('Status stale'),{timeout:5000});
  for(const id of ['capture','start','pause','resume','clear_goal','complete','cancel'])assert.equal(await page.getByTestId(id).isDisabled(),true,id);await page.close();
});
test('odom plan and narrow screens render without horizontal document overflow',async()=>{
  const page=await browser.newPage({viewport:{width:1280,height:1000}});await page.goto(base+'?fixture=odom');await page.waitForFunction(()=>document.querySelector('.ig-pathwrap').parentElement.textContent.includes('go2_odom'));
  await page.screenshot({path:join(directory,'preview-odom.png'),fullPage:true});await page.setViewportSize({width:600,height:1050});await page.waitForTimeout(350);
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);await page.screenshot({path:join(directory,'preview-mobile.png'),fullPage:true});await page.close();
});
test('runtime-provided HTML is always displayed as text',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=stale');await page.evaluate(()=>globalThis.imagegoalDemo.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify({session_id:'s',epoch:0,state:'IDLE',gates:['<img src=x onerror="window.injected=1">']})}}]));assert.equal(await page.locator('.ig-gates img').count(),0);assert.match(await page.locator('.ig-gates').innerText(),/<img/);assert.equal(await page.evaluate(()=>globalThis.injected),undefined);await page.close();
});

test('oversize upload is rejected before it becomes a draft or publishes a command',async()=>{
  const page=await browser.newPage();await page.goto(base);const image=Buffer.alloc(4*1024*1024+1);image[0]=255;image[1]=216;
  await page.getByTestId('goal-file').setInputFiles({name:'large.jpg',mimeType:'image/jpeg',buffer:image});await page.waitForFunction(()=>document.querySelector('[data-testid="action-status"]').textContent.includes('4 MB'));
  assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal((await page.evaluate(()=>globalThis.imagegoalDemo.published)).length,0);await page.close();
});
test('delayed lower epoch status cannot roll back acknowledged state',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=stale');await page.getByTestId('pause').click();await page.waitForFunction(()=>document.querySelector('.ig-state').textContent==='Paused');
  await page.evaluate(()=>globalThis.imagegoalDemo.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify({session_id:'demo-session-20261016',epoch:7,state:'CANCELLED',timestamp_ns:Date.now()*1e6,task:{task_id:'demo-task-008'}})}}]));
  assert.equal(await page.locator('.ig-state').innerText(),'Paused');assert.match(await page.locator('.ig-idgrid').innerText(),/epoch 8/);await page.close();
});

test('capture freezes exact bytes, RGB keeps updating, and recapture returns to live with start disabled',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.getByTestId('capture').click();await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);
  assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.some(set=>set.some(v=>v.topic.endsWith('/goal/compressed')))),true);
  assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).some(v=>v.topic.endsWith('/goal/compressed'))),false);
  const original=await page.getByTestId('goal').getAttribute('src');
  await page.evaluate(()=>{const c=document.createElement('canvas');c.width=100;c.height=100;const x=c.getContext('2d');x.fillStyle='red';x.fillRect(0,0,100,100);globalThis.imagegoalDemo.setRgb(c.toDataURL('image/jpeg'));});
  await page.waitForTimeout(450);assert.equal(await page.getByTestId('goal').getAttribute('src'),original);assert.notEqual(await page.getByTestId('rgb').getAttribute('src'),original);
  await page.getByTestId('start').click();await page.waitForFunction(()=>document.querySelector('[data-testid="action-status"]').textContent.includes('confirmed by runtime'));
  await page.getByTestId('clear_goal').click();await page.waitForFunction(()=>document.querySelector('[data-testid="goal-mode"]').textContent.includes('no goal captured'));
  assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('goal').getAttribute('src'),await page.getByTestId('rgb').getAttribute('src'));assert.equal(await page.locator('.ig-state').innerText(),'Stopped');
  const commands=await page.evaluate(()=>globalThis.imagegoalDemo.published);assert.equal(commands.at(-1).action,'clear_goal');assert.equal(commands.at(-1).payload.expected_epoch,9);await page.close();
});

test('external JPEG is committed and acknowledged before start becomes available',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.evaluate(()=>{globalThis.imagegoalDemo.controls.ackDelayMs=450;globalThis.imagegoalDemo.controls.suppressTicks=true;});
  const jpeg=await readFile(join(directory,'live-rgb.jpg'));await page.getByTestId('goal-file').setInputFiles({name:'goal.jpg',mimeType:'image/jpeg',buffer:jpeg});
  await page.waitForFunction(()=>globalThis.imagegoalDemo.published.some(c=>c.action==='import_goal'));assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('cancel').isDisabled(),false);
  await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);
  const commands=await page.evaluate(()=>globalThis.imagegoalDemo.published);assert.equal(commands[0].payload.expected_epoch,7);assert.equal(commands[0].payload.goal.rgb_jpeg_base64,jpeg.toString('base64'));
  const displayed=await page.getByTestId('goal').evaluate(async img=>Array.from(new Uint8Array(await(await fetch(img.src)).arrayBuffer())));assert.deepEqual(Buffer.from(displayed),jpeg);
  await page.getByTestId('start').click();await page.waitForFunction(()=>globalThis.imagegoalDemo.published.some(c=>c.action==='start'));assert.equal((await page.evaluate(()=>globalThis.imagegoalDemo.published)).at(-1).payload.selection_id,'selection-1');await page.close();
});

test('stop supersedes a delayed capture ACK without installing its image',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.evaluate(()=>{globalThis.imagegoalDemo.controls.ackDelayMs=500;globalThis.imagegoalDemo.controls.suppressTicks=true;});
  await page.getByTestId('capture').click();await page.evaluate(()=>{globalThis.imagegoalDemo.controls.ackDelayMs=80;});await page.getByTestId('cancel').click();
  await page.waitForFunction(()=>document.querySelector('[data-testid="action-status"]').textContent.includes('Stop confirmed by runtime'));await page.waitForTimeout(600);
  assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('goal').getAttribute('src'),null);assert.equal(await page.locator('.ig-state').innerText(),'Stopped');await page.close();
});

test('stop invalidates an import while its local file read is still pending',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.evaluate(()=>{globalThis.imagegoalDemo.controls.suppressTicks=true;const original=File.prototype.arrayBuffer;File.prototype.arrayBuffer=async function(){await new Promise(resolve=>setTimeout(resolve,500));return await original.call(this);};});
  await page.getByTestId('goal-file').setInputFiles({name:'goal.jpg',mimeType:'image/jpeg',buffer:await readFile(join(directory,'live-rgb.jpg'))});await page.getByTestId('cancel').click();
  await page.waitForFunction(()=>document.querySelector('[data-testid="action-status"]').textContent.includes('Stop confirmed by runtime'));await page.waitForTimeout(600);
  assert.deepEqual((await page.evaluate(()=>globalThis.imagegoalDemo.published)).map(c=>c.action),['cancel']);assert.equal(await page.getByTestId('start').isDisabled(),true);assert.match(await page.getByTestId('goal-mode').innerText(),/no goal captured/);await page.close();
});

test('session and camera changes discard old capture ACKs and do not mix native RGB sources',async()=>{
  for(const change of ['session','camera']){
    const page=await browser.newPage();await page.goto(base);await page.evaluate(()=>{globalThis.imagegoalDemo.controls.ackDelayMs=450;globalThis.imagegoalDemo.controls.suppressTicks=true;});await page.getByTestId('capture').click();
    await page.evaluate(change=>{const d=globalThis.imagegoalDemo,s=d.demoStatus();s.goal_selection={mode:'live',ready:false};s.timestamp_ns=Date.now()*1e6+1000;if(change==='session')s.session_id='new-session';s.sensors.rgb={age_s:.02,frame_name:'go2_front_camera_optical_frame',timestamp_source:'go2_video_rpc_receive_host_system_time_not_sensor_source_time'};d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);d.emit([{topic:'/imagegoal/shadow/rgb/compressed',message:d.image('data:image/jpeg;base64,/9j/','camera_color_optical_frame')}]);},change);
    assert.equal(await page.getByTestId('rgb').getAttribute('src'),null);assert.equal(await page.getByTestId('goal').getAttribute('src'),null);
    await page.evaluate(()=>{const d=globalThis.imagegoalDemo,c=document.createElement('canvas');c.width=100;c.height=100;const x=c.getContext('2d');x.fillStyle='green';x.fillRect(0,0,100,100);d.emit([{topic:'/imagegoal/shadow/rgb/compressed',message:d.image(c.toDataURL('image/jpeg'),'go2_front_camera_optical_frame')}]);});
    await page.waitForFunction(()=>document.querySelector('[data-testid="rgb"]').naturalWidth>0);assert.equal(await page.getByTestId('rgb').getAttribute('src'),await page.getByTestId('goal').getAttribute('src'));assert.match(await page.getByTestId('preview-source-age').innerText(),/RPC receipt/);
    await page.waitForTimeout(550);assert.equal(await page.getByTestId('start').isDisabled(),true);assert.match(await page.getByTestId('goal-mode').innerText(),/no goal captured/);await page.close();
  }
});

test('reconnected frozen selection waits for its matching image and rejects old selection images',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.evaluate(()=>{const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const s=d.demoStatus();s.epoch=20;s.goal_selection={mode:'frozen',ready:true,selection_id:'reconnected-goal',source:'import'};d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);const c=document.createElement('canvas');c.width=64;c.height=64;const data=c.toDataURL('image/jpeg');d.emit([{topic:'/imagegoal/shadow/goal/compressed',message:d.image(data,'imagegoal_frozen_goal_old-goal')}]);});
  assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('goal').getAttribute('src'),null);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo,c=document.createElement('canvas');c.width=64;c.height=64;d.emit([{topic:'/imagegoal/shadow/goal/compressed',message:d.image(c.toDataURL('image/jpeg'),'imagegoal_frozen_goal_reconnected-goal')}]);});
  await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);await page.getByTestId('start').click();const command=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).at(-1);assert.equal(command.payload.selection_id,'reconnected-goal');assert.equal(command.payload.expected_epoch,20);await page.close();
});

test('exactly one video topic is subscribed, including when the active camera is native Go2',async()=>{
  const page=await browser.newPage();await page.goto(base);
  const subscriptions=await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.flat().map(v=>v.topic));
  assert.deepEqual(subscriptions.filter(topic=>topic.includes('rgb/compressed')||topic.includes('image_raw/compressed')),['/imagegoal/shadow/rgb/compressed']);
  assert.equal(subscriptions.includes('/go2/front_camera/image_raw/compressed'),false);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const s=d.demoStatus();s.sensors.rgb={age_s:.02,frame_name:'go2_front_camera_optical_frame',timestamp_source:'go2_video_rpc_receive_host_system_time_not_sensor_source_time'};d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);const c=document.createElement('canvas');c.width=24;c.height=24;d.emit([{topic:'/go2/front_camera/image_raw/compressed',message:d.image(c.toDataURL('image/jpeg'),'go2_front_camera_optical_frame')}]);});
  assert.equal(await page.getByTestId('rgb').getAttribute('src'),null);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo,c=document.createElement('canvas');c.width=24;c.height=24;d.emit([{topic:'/imagegoal/shadow/rgb/compressed',message:d.image(c.toDataURL('image/jpeg'),'go2_front_camera_optical_frame')}]);});
  await page.waitForFunction(()=>document.querySelector('[data-testid="rgb"]').naturalWidth===24);assert.match(await page.getByTestId('preview-source-age').innerText(),/RPC receipt time/);await page.close();
});

test('frozen JPEG is subscribed only until matching bytes decode, with live channels kept throughout',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.evaluate(()=>{globalThis.imagegoalDemo.controls.suppressTicks=true;});
  const live=['/imagegoal/shadow/status','/imagegoal/shadow/bridge_status','/imagegoal/operator/result','/imagegoal/shadow/rgb/compressed'];
  assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).map(v=>v.topic)),live);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo,s=d.demoStatus();s.epoch=20;s.goal_selection={mode:'frozen',ready:true,selection_id:'fetched-goal',source:'import'};d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);d.emit([{topic:'/imagegoal/shadow/goal/compressed',message:{format:'jpeg',header:{frame_id:'imagegoal_frozen_goal_fetched-goal'},data:new Uint8Array([255,216,255,217])}}]);});
  await page.waitForFunction(()=>document.querySelector('[data-testid="action-status"]').textContent.includes('could not be decoded'));
  assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).map(v=>v.topic)),[...live,'/imagegoal/shadow/goal/compressed']);
  assert.equal(await page.getByTestId('start').isDisabled(),true);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo,c=document.createElement('canvas');c.width=40;c.height=30;d.emit([{topic:'/imagegoal/shadow/goal/compressed',message:d.image(c.toDataURL('image/jpeg'),'imagegoal_frozen_goal_fetched-goal')}]);});
  await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);
  assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).map(v=>v.topic)),live);
  const subscriptionCount=await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.length);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo,s=d.demoStatus();s.epoch=20;s.goal_selection={mode:'frozen',ready:true,selection_id:'fetched-goal',source:'import'};for(let i=0;i<10;i++){s.timestamp_ns=Date.now()*1e6+i;d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);}});
  await page.waitForTimeout(400);assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.length),subscriptionCount);
  assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.every(set=>['/imagegoal/shadow/status','/imagegoal/shadow/bridge_status','/imagegoal/operator/result','/imagegoal/shadow/rgb/compressed'].every(topic=>set.some(v=>v.topic===topic)))),true);
  await page.close();
});

test('replacement, clear and session changes renew frozen image fetch without using stale bytes',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.evaluate(()=>{globalThis.imagegoalDemo.controls.suppressTicks=true;});
  async function status(selectionId,session='demo-session-20261016'){
    await page.evaluate(({selectionId,session})=>{const d=globalThis.imagegoalDemo,s=d.demoStatus();s.epoch=30;s.session_id=session;s.goal_selection=selectionId?{mode:'frozen',ready:true,selection_id:selectionId,source:'import'}:{mode:'live',ready:false};d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);},{selectionId,session});
  }
  async function goal(selectionId){await page.evaluate(selectionId=>{const d=globalThis.imagegoalDemo,c=document.createElement('canvas');c.width=40;c.height=30;d.emit([{topic:'/imagegoal/shadow/goal/compressed',message:d.image(c.toDataURL('image/jpeg'),`imagegoal_frozen_goal_${selectionId}`)}]);},selectionId);}
  async function fetching(){return await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).some(v=>v.topic==='/imagegoal/shadow/goal/compressed'));}
  await status('first');assert.equal(await fetching(),true);await goal('first');await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);assert.equal(await fetching(),false);
  await status('replacement');assert.equal(await fetching(),true);await goal('first');assert.equal(await page.getByTestId('start').isDisabled(),true);await goal('replacement');await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);assert.equal(await fetching(),false);
  await status('replacement','new-session');assert.equal(await fetching(),true);assert.equal(await page.getByTestId('goal').getAttribute('src'),null);assert.equal(await page.getByTestId('start').isDisabled(),true);
  await status(undefined,'new-session');assert.equal(await fetching(),false);
  await status('after-clear','new-session');assert.equal(await fetching(),true);await goal('after-clear');await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);assert.equal(await fetching(),false);
  await page.close();
});

test('language navigation and tracking never fetch goals and return to the frozen NoMaD selection intact',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.evaluate(()=>{globalThis.imagegoalDemo.controls.suppressTicks=true;const d=globalThis.imagegoalDemo,s=d.demoStatus();s.epoch=20;s.goal_selection={mode:'frozen',ready:true,selection_id:'preserved',source:'import'};d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);});
  assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).some(v=>v.topic.endsWith('/goal/compressed'))),true);
  for(const mode of ['objnav','vln','tracking']){
    if(mode==='tracking')await page.getByTestId('tracking').click();else{await page.getByTestId('lightnav').click();await page.getByTestId('navigation-mode').selectOption(mode);}
    assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).some(v=>v.topic.endsWith('/goal/compressed'))),false);
    await page.evaluate(()=>{const d=globalThis.imagegoalDemo,c=document.createElement('canvas');c.width=40;c.height=30;d.emit([{topic:'/imagegoal/shadow/goal/compressed',message:d.image(c.toDataURL('image/jpeg'),'imagegoal_frozen_goal_preserved')}]);});
    assert.equal(await page.getByTestId('goal').getAttribute('src'),null);assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.published),[]);
  }
  await page.getByTestId('nomad').click();assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).some(v=>v.topic.endsWith('/goal/compressed'))),true);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo,c=document.createElement('canvas');c.width=40;c.height=30;d.emit([{topic:'/imagegoal/shadow/goal/compressed',message:d.image(c.toDataURL('image/jpeg'),'imagegoal_frozen_goal_preserved')}]);});
  await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);const frozen=await page.getByTestId('goal').getAttribute('src');
  await page.getByTestId('tracking').click();await page.getByTestId('nomad').click();assert.equal(await page.getByTestId('goal').getAttribute('src'),frozen);assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).some(v=>v.topic.endsWith('/goal/compressed'))),false);assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.published),[]);
  await page.close();
});

test('language modes decode only the visible camera and restore live goal preview on return',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.waitForFunction(()=>document.querySelector('[data-testid="rgb"]').naturalWidth>0);await page.evaluate(()=>{globalThis.imagegoalDemo.controls.suppressTicks=true;});
  let width=40;
  for(const mode of ['objnav','vln','tracking']){
    if(mode==='tracking')await page.getByTestId('tracking').click();else{await page.getByTestId('lightnav').click();await page.getByTestId('navigation-mode').selectOption(mode);}
    await page.evaluate(width=>{const d=globalThis.imagegoalDemo,original=HTMLImageElement.prototype.decode;globalThis.goalDecodeQA={original,calls:[]};HTMLImageElement.prototype.decode=function(){globalThis.goalDecodeQA.calls.push(this.dataset.testid);return original.call(this);};const c=document.createElement('canvas');c.width=width;c.height=30;d.emit([{topic:'/imagegoal/shadow/rgb/compressed',message:d.image(c.toDataURL('image/jpeg'))}]);},width);
    await page.waitForFunction(width=>document.querySelector('[data-testid="rgb"]').naturalWidth===width,width);
    assert.deepEqual(await page.evaluate(()=>globalThis.goalDecodeQA.calls),['rgb']);await page.evaluate(()=>{HTMLImageElement.prototype.decode=globalThis.goalDecodeQA.original;});width++;
  }
  await page.getByTestId('nomad').click();await page.waitForFunction(()=>document.querySelector('[data-testid="goal"]').getAttribute('src')===document.querySelector('[data-testid="rgb"]').getAttribute('src'));assert.equal(await page.getByTestId('goal').evaluate(img=>img.naturalWidth),width-1);assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.published),[]);await page.close();
});

test('new frames decode offscreen and replace camera/live-goal atomically before old URLs are revoked',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.waitForFunction(()=>document.querySelector('[data-testid="rgb"]').naturalWidth>0&&document.querySelector('[data-testid="goal"]').naturalWidth>0);
  const before=await page.getByTestId('rgb').getAttribute('src');
  await page.evaluate(()=>{
    const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const originalDecode=HTMLImageElement.prototype.decode,originalRevoke=URL.revokeObjectURL;
    const qa={resolvers:[],revoked:[],badRevokes:[],originalDecode,originalRevoke};globalThis.frameQA=qa;
    HTMLImageElement.prototype.decode=function(){return originalDecode.call(this).then(()=>new Promise(resolve=>qa.resolvers.push(resolve)));};
    URL.revokeObjectURL=function(url){qa.revoked.push(url);if([...document.querySelectorAll('[data-testid="rgb"],[data-testid="goal"]')].some(img=>img.getAttribute('src')===url))qa.badRevokes.push(url);return originalRevoke.call(this,url);};
    const c=document.createElement('canvas');c.width=48;c.height=48;c.getContext('2d').fillRect(0,0,48,48);d.emit([{topic:'/imagegoal/shadow/rgb/compressed',message:d.image(c.toDataURL('image/jpeg'))}]);
  });
  await page.waitForFunction(()=>globalThis.frameQA.resolvers.length===2);
  assert.equal(await page.getByTestId('rgb').getAttribute('src'),before);assert.equal(await page.getByTestId('goal').getAttribute('src'),before);
  assert.equal(await page.locator('.ig-imagebox.has-image').count(),2);assert.equal((await page.evaluate(()=>globalThis.frameQA.revoked)).includes(before),false);
  await page.evaluate(()=>{HTMLImageElement.prototype.decode=globalThis.frameQA.originalDecode;globalThis.frameQA.resolvers.splice(0).forEach(resolve=>resolve());});
  await page.waitForFunction(old=>document.querySelector('[data-testid="rgb"]').getAttribute('src')!==old,before);
  assert.equal(await page.getByTestId('rgb').getAttribute('src'),await page.getByTestId('goal').getAttribute('src'));assert.equal(await page.getByTestId('rgb').evaluate(img=>img.naturalWidth),48);
  assert.equal((await page.evaluate(()=>globalThis.frameQA.revoked)).includes(before),true);assert.deepEqual(await page.evaluate(()=>globalThis.frameQA.badRevokes),[]);
  await page.close();
});

test('failed frame decoding preserves both last good images and status updates do not reset their sources',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.waitForFunction(()=>document.querySelector('[data-testid="rgb"]').naturalWidth>0&&document.querySelector('[data-testid="goal"]').naturalWidth>0);
  const before=await page.getByTestId('rgb').getAttribute('src');
  const result=await page.evaluate(async()=>{
    const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const sourceChanges=[];const observer=new MutationObserver(rows=>sourceChanges.push(...rows.filter(row=>row.attributeName==='src')));observer.observe(document.querySelector('[data-testid="goal"]'),{attributes:true});
    for(let i=0;i<10;i++)d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(d.demoStatus())}}]);
    await Promise.resolve();observer.disconnect();d.emit([{topic:'/imagegoal/shadow/rgb/compressed',message:{format:'jpeg',data:new Uint8Array([255,216,255,217])}}]);
    return sourceChanges.length;
  });
  assert.equal(result,0);await page.waitForTimeout(100);assert.equal(await page.getByTestId('rgb').getAttribute('src'),before);assert.equal(await page.getByTestId('goal').getAttribute('src'),before);assert.equal(await page.locator('.ig-imagebox.has-image').count(),2);await page.close();
});

test('camera decoding keeps only one newest pending frame instead of building a backlog',async()=>{
  const page=await browser.newPage();await page.goto(base);await page.waitForFunction(()=>document.querySelector('[data-testid="rgb"]').naturalWidth>0&&document.querySelector('[data-testid="goal"]').naturalWidth>0);
  await page.evaluate(()=>{
    const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const originalDecode=HTMLImageElement.prototype.decode,originalCreate=URL.createObjectURL;const qa={resolvers:[],created:0,originalDecode,originalCreate};globalThis.pendingQA=qa;
    HTMLImageElement.prototype.decode=function(){return originalDecode.call(this).then(()=>new Promise(resolve=>qa.resolvers.push(resolve)));};URL.createObjectURL=function(...args){qa.created++;return originalCreate.apply(this,args);};
    for(let n=1;n<=20;n++){const c=document.createElement('canvas');c.width=n+20;c.height=24;d.emit([{topic:'/imagegoal/shadow/rgb/compressed',message:d.image(c.toDataURL('image/jpeg'))}]);}
  });
  await page.waitForFunction(()=>globalThis.pendingQA.resolvers.length===2);assert.equal(await page.evaluate(()=>globalThis.pendingQA.created),1);
  await page.evaluate(()=>{HTMLImageElement.prototype.decode=globalThis.pendingQA.originalDecode;globalThis.pendingQA.resolvers.splice(0).forEach(resolve=>resolve());});
  await page.waitForFunction(()=>document.querySelector('[data-testid="rgb"]').naturalWidth===40);assert.equal(await page.evaluate(()=>globalThis.pendingQA.created),2);await page.close();
});

test('both native buttons preserve selected backend and instruction while shadow start remains shadow',async()=>{
  for(const backend of ['nomad','lightnav']){
    const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId(backend).click();
    assert.equal(await page.getByTestId('native_start').isDisabled(),true);
    if(backend==='nomad'){await page.getByTestId('capture').click();}else{await page.getByTestId('instruction').fill('Go to the coat rack.');}
    await page.waitForFunction(()=>!document.querySelector('[data-testid="native_start"]').disabled);
    const label=backend==='nomad'?'NoMaD':'LightNav';assert.equal(await page.getByTestId('native_start').innerText(),`Run ${label} on robot`);
    await page.getByTestId('native_start').click();await page.waitForFunction(()=>document.querySelector('[data-testid="native-state"]').textContent.includes('process running'));
    const request=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).find(c=>c.action==='native_start');
    assert.deepEqual(request.payload,backend==='nomad'?{expected_session_id:'demo-session-20261016',expected_epoch:8,selection_id:'selection-1',backend,instruction:'',timeout_s:60,operator_present:true}:{expected_session_id:'demo-session-20261016',expected_epoch:7,backend,task_mode:'objnav',instruction:'Go to the coat rack.',timeout_s:60,operator_present:true});
    assert.match(await page.locator('.ig-banner').innerText(),new RegExp(`NATIVE · ${label} robot run`));assert.match(await page.getByTestId('native-state').innerText(),/process running.*not granted/);
    await page.getByTestId(backend==='nomad'?'lightnav':'nomad').click();assert.match(await page.locator('.ig-banner').innerText(),new RegExp(`Active ${label} robot run`));
    assert.match(await page.getByTestId('native-state').innerText(),new RegExp(`^${label} Executor`));
    await page.getByTestId(backend).click();await page.getByTestId('native_stop').click();await page.waitForFunction(()=>document.querySelector('.ig-state').textContent==='Stopped');
    await page.getByTestId('start').click();await page.waitForFunction(()=>globalThis.imagegoalDemo.published.some(c=>c.action==='start'));
    const shadow=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).find(c=>c.action==='start');assert.equal(shadow.payload.execution_mode,'shadow');
    await page.close();
  }
});
test('LightNav native stop supersedes a delayed start acknowledgement and ordinary Stop uses native stop',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('lightnav').click();await page.getByTestId('instruction').fill('Go to the coat rack.');await page.waitForFunction(()=>!document.querySelector('[data-testid="native_start"]').disabled);
  await page.evaluate(()=>{globalThis.imagegoalDemo.controls.ackDelayMs=500;globalThis.imagegoalDemo.controls.suppressTicks=true;});
  await page.getByTestId('native_start').click();assert.equal(await page.getByTestId('native_stop').isDisabled(),false);
  await page.evaluate(()=>{globalThis.imagegoalDemo.controls.ackDelayMs=40;});await page.getByTestId('cancel').click();
  await page.waitForFunction(()=>document.querySelector('.ig-state').textContent==='Stopped');await page.waitForTimeout(550);
  assert.deepEqual((await page.evaluate(()=>globalThis.imagegoalDemo.published)).map(c=>c.action),['native_start','native_stop']);
  assert.match(await page.getByTestId('native-state').innerText(),/stopped/);assert.equal(await page.getByTestId('native_start').isDisabled(),false);await page.close();
});
test('old NoMaD-only runtime never enables the LightNav native button',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('capture').click();await page.waitForFunction(()=>!document.querySelector('[data-testid="native_start"]').disabled);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const s=d.demoStatus();delete s.execution_capability.supervised_native_backends;d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);});
  await page.getByTestId('lightnav').click();assert.equal(await page.getByTestId('native_start').isDisabled(),true);assert.match(await page.getByTestId('native-reason').innerText(),/LightNav.*unavailable/);
  await page.getByTestId('nomad').click();assert.equal(await page.getByTestId('native_start').isDisabled(),false);assert.equal((await page.evaluate(()=>globalThis.imagegoalDemo.published)).some(c=>c.action==='native_start'),false);await page.close();
});

test('one-button native tracking starts without a goal, never auto-starts on mode changes, and uses priority stop',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('tracking').click();
  assert.equal(await page.getByTestId('goal').isVisible(),false);assert.equal(await page.getByTestId('native_start').isDisabled(),false);assert.equal(await page.getByTestId('native_start').innerText(),'Run LightNav tracking');
  assert.equal(await page.getByTestId('instruction').inputValue(),'Follow the person in the white T-shirt.');assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.published),[]);
  await page.getByTestId('native_start').click();assert.equal(await page.getByTestId('native_start').isDisabled(),true);await page.waitForFunction(()=>document.querySelector('[data-testid="native-state"]').textContent.includes('process running'));
  const request=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).find(c=>c.action==='native_start');
  assert.deepEqual(request.payload,{expected_session_id:'demo-session-20261016',expected_epoch:7,backend:'lightnav',task_mode:'tracking',instruction:'Follow the person in the white T-shirt.',timeout_s:60,operator_present:true});
  assert.match(await page.locator('.ig-banner').innerText(),/NATIVE · LightNav tracking/);assert.equal(await page.getByTestId('native_start').isDisabled(),true);assert.equal(await page.getByTestId('start').isDisabled(),true);
  await page.screenshot({path:join(directory,'preview-tracking.png'),fullPage:true});
  await page.getByTestId('nomad').click();assert.match(await page.locator('.ig-banner').innerText(),/Active LightNav tracking/);assert.equal((await page.evaluate(()=>globalThis.imagegoalDemo.published)).length,1);
  await page.getByTestId('cancel').click();await page.waitForFunction(()=>document.querySelector('.ig-state').textContent==='Stopped');assert.equal((await page.evaluate(()=>globalThis.imagegoalDemo.published)).at(-1).action,'native_stop');
  await page.getByTestId('tracking').click();await page.getByTestId('start').click();await page.waitForFunction(()=>globalThis.imagegoalDemo.published.some(c=>c.action==='start'));
  const shadow=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).find(c=>c.action==='start');assert.equal(shadow.payload.task_mode,'tracking');assert.equal(shadow.payload.execution_mode,'shadow');assert.equal(shadow.payload.selection_id,undefined);assert.equal(shadow.payload.goal,undefined);await page.close();
});
test('tracking preserves separate target text, requires explicit capabilities, and never bypasses stale or read-only status',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('lightnav').click();await page.getByTestId('instruction').fill('Go to the coat rack.');await page.getByTestId('tracking').click();await page.getByTestId('instruction').fill('Follow the person in blue.');
  await page.getByTestId('lightnav').click();assert.equal(await page.getByTestId('instruction').inputValue(),'Go to the coat rack.');await page.getByTestId('tracking').click();assert.equal(await page.getByTestId('instruction').inputValue(),'Follow the person in blue.');
  await page.getByTestId('instruction').fill('   ');assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('native_start').isDisabled(),true);assert.match(await page.getByTestId('native-reason').innerText(),/Enter/);await page.getByTestId('instruction').fill('Follow the person.');
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const s=d.demoStatus();delete s.execution_capability.supervised_native_task_modes;delete s.execution_capability.task_modes;d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);});
  assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('native_start').isDisabled(),true);assert.match(await page.getByTestId('native-reason').innerText(),/LightNav tracking.*unavailable/);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo;d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(d.demoStatus())}}]);});assert.equal(await page.getByTestId('native_start').isDisabled(),false);
  await page.waitForFunction(()=>document.querySelector('[data-testid="connection"]').textContent.includes('Status stale'),{timeout:5000});assert.equal(await page.getByTestId('native_start').isDisabled(),true);assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('native_stop').isDisabled(),false);
  assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.published),[]);await page.close();
  const snapshot=await browser.newPage();await snapshot.goto(base+'?fixture=live');assert.equal(await snapshot.getByTestId('tracking').isDisabled(),true);assert.equal(await snapshot.getByTestId('native_start').isDisabled(),true);await snapshot.close();
});
test('tracking distinguishes visible, unknown and safe hold without claiming identity recovery',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('tracking').click();await page.getByTestId('native_start').click();await page.waitForFunction(()=>document.querySelector('[data-testid="native-state"]').textContent.includes('process running'));
  await page.evaluate(()=>{globalThis.imagegoalDemo.controls.suppressTicks=true;});
  for(const [visible,targetState,state,label] of [[true,'visible','TRACKING','Target visible'],[null,'unknown','WAITING_INPUT','Visibility unknown'],[false,'holding','TRACK_HOLD','Motion held · observing']]){
    await page.evaluate(({visible,targetState,state})=>{const d=globalThis.imagegoalDemo,s=d.demoStatus();s.state=state;s.tracking={enabled:true,instruction:'Follow the person in white.',visible,target_state:targetState,hold_reason:visible===false?'target_not_visible':null};d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);},{visible,targetState,state});
    assert.match(await page.getByTestId('tracking-visibility').innerText(),new RegExp(label));assert.match(await page.getByTestId('tracking-target').innerText(),/Active target: Follow the person in white/);
  }
  assert.equal((await page.evaluate(()=>globalThis.imagegoalDemo.published)).length,1);assert.match(await page.locator('.ig-state').innerText(),/Tracking held/);await page.close();
});
test('tracking Stop supersedes a delayed native start acknowledgement',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('tracking').click();await page.evaluate(()=>{globalThis.imagegoalDemo.controls.ackDelayMs=500;globalThis.imagegoalDemo.controls.suppressTicks=true;});
  await page.getByTestId('native_start').click();await page.evaluate(()=>{globalThis.imagegoalDemo.controls.ackDelayMs=30;});await page.getByTestId('native_stop').click();await page.waitForFunction(()=>document.querySelector('.ig-state').textContent==='Stopped');await page.waitForTimeout(550);
  assert.deepEqual((await page.evaluate(()=>globalThis.imagegoalDemo.published)).map(c=>c.action),['native_start','native_stop']);assert.match(await page.getByTestId('native-state').innerText(),/stopped/);assert.equal(await page.getByTestId('native_start').isDisabled(),false);await page.close();
});

test('official ObjNav/VLN starts without a goal, requires text, and never sends goal pixels or selection',async()=>{
  for(const mode of ['objnav','vln']){
    const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('lightnav').click();
    assert.equal(await page.getByTestId('goal').isVisible(),false);assert.equal(await page.getByTestId('native_start').isDisabled(),true);assert.equal(await page.getByTestId('start').isDisabled(),true);
    assert.match(await page.getByTestId('native-reason').innerText(),/Enter a navigation instruction/);assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.published),[]);
    await page.getByTestId('navigation-mode').selectOption(mode);await page.getByTestId('instruction').fill('  Go to the coat rack.  ');
    await page.getByTestId('native_start').click();await page.waitForFunction(()=>document.querySelector('[data-testid="native-state"]').textContent.includes('process running'));
    const native=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).at(-1);
    assert.equal(native.payload.task_mode,mode);assert.equal(native.payload.instruction,'Go to the coat rack.');assert.equal(native.payload.selection_id,undefined);assert.equal(native.payload.goal,undefined);
    await page.getByTestId('native_stop').click();await page.waitForFunction(()=>document.querySelector('.ig-state').textContent==='Stopped');
    await page.getByTestId('start').click();await page.waitForFunction(()=>globalThis.imagegoalDemo.published.some(c=>c.action==='start'));
    const shadow=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).at(-1);assert.equal(shadow.payload.task_mode,mode);assert.equal(shadow.payload.execution_mode,'shadow');assert.equal(shadow.payload.selection_id,undefined);assert.equal(shadow.payload.goal,undefined);
    assert.match(await page.locator('.ig-meta').allTextContents().then(v=>v.join(' ')),new RegExp(mode.toUpperCase()));await page.close();
  }
});

test('HTTP operator submits official ObjNav/VLN without any image and keeps backend in native mode',async()=>{
  for(const mode of ['objnav','vln'])for(const native of [false,true]){
    const page=await browser.newPage(),requests=[];
    const status={session_id:'http-session',epoch:3,state:'IDLE',task:null,sensors:{},execution_capability:{supervised_native_configured:true,operator_supervised:true,supervised_native_backends:['nomad','lightnav'],task_modes:['imagegoal','objnav','vln','tracking'],supervised_native_task_modes:['imagegoal','objnav','vln','tracking']}};
    await page.route('**/api/**',async route=>{const req=route.request(),path=new URL(req.url()).pathname;if(req.method()==='POST')requests.push({path,body:req.postDataJSON()});await route.fulfill({status:path.endsWith('.jpg')?404:200,contentType:'application/json',body:JSON.stringify(path==='/api/status'?status:path.startsWith('/api/events')?{events:[]}:{ok:true})});});
    await page.goto(base+'/web/index.html');await page.waitForFunction(()=>document.querySelector('#connection').textContent==='已连接');
    await page.locator('#backend').selectOption('lightnav');await page.locator('#navigation-mode').selectOption(mode);
    assert.equal(await page.locator('#goal-preview').isVisible(),false);assert.equal(await page.locator('#goal-controls').isVisible(),false);assert.equal(await page.locator('#start').isDisabled(),true);
    if(native)await page.locator('#execution-mode').selectOption('supervised_native');
    assert.equal(await page.locator('#backend').inputValue(),'lightnav');await page.locator('#instruction').fill('  Go to the coat rack.  ');assert.equal(await page.locator('#start').isDisabled(),false);
    await page.locator('#start').click();await page.waitForFunction(()=>document.querySelector('#action-status').textContent.includes('操作已提交'));
    assert.equal(requests.length,1);assert.equal(requests[0].path,native?'/api/native/start':'/api/tasks');
    assert.deepEqual(requests[0].body,{task_mode:mode,backend:'lightnav',execution_mode:native?'supervised_native':'shadow',timeout_s:native?60:120,instruction:'Go to the coat rack.',...(native?{operator_present:true}:{}),expected_epoch:3,expected_session_id:'http-session'});
    await page.close();
  }
});

test('LightNav ImageGoal adaptation freezes camera/imported goals, keeps optional text, and preserves language isolation',async()=>{
  for(const source of ['capture','import']){
    const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('lightnav-imagegoal').click();
    assert.equal(await page.getByTestId('goal').isVisible(),true);assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('native_start').isDisabled(),true);
    assert.match(await page.getByTestId('imagegoal-adaptation-note').innerText(),/Go2 ImageGoal adapter.*experimental/);assert.equal(await page.getByTestId('instruction').inputValue(),'');
    assert.deepEqual(await page.evaluate(()=>globalThis.imagegoalDemo.published),[]);
    if(source==='capture')await page.getByTestId('capture').click();else await page.getByTestId('goal-file').setInputFiles({name:'goal.jpg',mimeType:'image/jpeg',buffer:await readFile(join(directory,'live-rgb.jpg'))});
    await page.waitForFunction(()=>!document.querySelector('[data-testid="native_start"]').disabled);
    assert.equal(await page.evaluate(()=>globalThis.imagegoalDemo.subscriptions.at(-1).some(v=>v.topic.endsWith('/goal/compressed'))),false);
    await page.getByTestId('native_start').click();await page.waitForFunction(()=>document.querySelector('[data-testid="native-state"]').textContent.includes('process running'));
    let commands=await page.evaluate(()=>globalThis.imagegoalDemo.published),native=commands.find(c=>c.action==='native_start');
    assert.deepEqual(native.payload,{expected_session_id:'demo-session-20261016',expected_epoch:8,selection_id:'selection-1',backend:'lightnav',task_mode:'imagegoal',instruction:'',timeout_s:60,operator_present:true});
    assert.equal(native.payload.goal,undefined);await page.getByTestId('native_stop').click();await page.waitForFunction(()=>document.querySelector('.ig-state').textContent==='Stopped');
    await page.getByTestId('instruction').fill('Approach the reference view.');await page.getByTestId('start').click();await page.waitForFunction(()=>globalThis.imagegoalDemo.published.some(c=>c.action==='start'));
    const shadow=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).find(c=>c.action==='start');assert.equal(shadow.payload.backend,'lightnav');assert.equal(shadow.payload.task_mode,'imagegoal');assert.equal(shadow.payload.selection_id,'selection-1');assert.equal(shadow.payload.instruction,'Approach the reference view.');assert.equal(shadow.payload.execution_mode,'shadow');assert.equal(shadow.payload.goal,undefined);
    await page.getByTestId('cancel').click();await page.waitForFunction(()=>document.querySelector('.ig-state').textContent==='Stopped');
    await page.getByTestId('lightnav').click();assert.equal(await page.getByTestId('instruction').inputValue(),'');assert.equal(await page.getByTestId('goal').isVisible(),false);await page.getByTestId('instruction').fill('Go to the window.');
    await page.getByTestId('start').click();await page.waitForFunction(()=>globalThis.imagegoalDemo.published.filter(c=>c.action==='start').length===2);
    const official=(await page.evaluate(()=>globalThis.imagegoalDemo.published)).at(-1);assert.equal(official.payload.task_mode,'objnav');assert.equal(official.payload.selection_id,undefined);assert.equal(official.payload.goal,undefined);assert.equal(official.payload.instruction,'Go to the window.');
    await page.getByTestId('lightnav-imagegoal').click();assert.equal(await page.getByTestId('instruction').inputValue(),'Approach the reference view.');await page.close();
  }
});
test('LightNav ImageGoal never enables against an old runtime even with an existing frozen goal',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');await page.getByTestId('capture').click();await page.waitForFunction(()=>!document.querySelector('[data-testid="start"]').disabled);
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const s=d.demoStatus();delete s.execution_capability.lightnav_imagegoal_adapter;d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);});
  await page.getByTestId('lightnav-imagegoal').click();assert.equal(await page.getByTestId('start').isDisabled(),true);assert.equal(await page.getByTestId('native_start').isDisabled(),true);assert.match(await page.getByTestId('native-reason').innerText(),/LightNav ImageGoal.*unavailable/);await page.close();
});
test('HTTP ImageGoal adaptation requires canonical camera/import freeze and sends its selection only',async()=>{
  for(const source of ['capture','import'])for(const native of [false,true]){
    const page=await browser.newPage(),requests=[],jpeg=await readFile(join(directory,'live-rgb.jpg'));
    const status={session_id:'http-imagegoal',epoch:3,state:'IDLE',task:null,sensors:{},goal_selection:{mode:'live',ready:false},execution_capability:{supervised_native_configured:true,operator_supervised:true,supervised_native_backends:['nomad','lightnav'],lightnav_imagegoal_adapter:true,task_modes:['imagegoal','objnav','vln','tracking'],supervised_native_task_modes:['imagegoal','objnav','vln','tracking']}};
    await page.route('**/api/**',async route=>{const req=route.request(),path=new URL(req.url()).pathname;if(req.method()==='POST')requests.push({path,body:req.postDataJSON()});
      if(path==='/api/goal/capture'||path==='/api/goal/import'){status.epoch++;status.goal_selection={mode:'frozen',ready:true,selection_id:'http-frozen',source};await route.fulfill({contentType:'application/json',body:JSON.stringify({status,selection:status.goal_selection,goal:{rgb_jpeg_base64:jpeg.toString('base64')}})});return;}
      if(path==='/api/goal.jpg'){await route.fulfill({contentType:'image/jpeg',body:jpeg});return;}
      await route.fulfill({status:path.endsWith('.jpg')?404:200,contentType:'application/json',body:JSON.stringify(path==='/api/status'?status:path.startsWith('/api/events')?{events:[]}:{ok:true})});});
    await page.goto(base+'/web/index.html');await page.waitForFunction(()=>document.querySelector('#connection').textContent==='已连接');await page.locator('#backend').selectOption('lightnav');await page.locator('#navigation-mode').selectOption('imagegoal');
    if(native)await page.locator('#execution-mode').selectOption('supervised_native');assert.equal(await page.locator('#goal-preview').isVisible(),true);assert.equal(await page.locator('#start').isDisabled(),true);
    if(source==='capture')await page.locator('#capture').click();else await page.locator('#goal-file').setInputFiles({name:'goal.jpg',mimeType:'image/jpeg',buffer:jpeg});
    await page.waitForFunction(()=>!document.querySelector('#start').disabled);await page.locator('#start').click();await page.waitForFunction(()=>document.querySelector('#action-status').textContent.includes('操作已提交'));
    assert.equal(requests[0].path,'/api/goal/'+source);assert.equal(requests[0].body.expected_epoch,3);
    assert.deepEqual(requests[1],{path:native?'/api/native/start':'/api/tasks',body:{task_mode:'imagegoal',selection_id:'http-frozen',backend:'lightnav',execution_mode:native?'supervised_native':'shadow',timeout_s:native?60:120,instruction:'',...(native?{operator_present:true}:{}),expected_epoch:4,expected_session_id:'http-imagegoal'}});
    await page.close();
  }
});

test('model STOP completes a LightNav task without claiming independently confirmed arrival',async()=>{
  const page=await browser.newPage();await page.goto(base+'?fixture=native');
  await page.evaluate(()=>{const d=globalThis.imagegoalDemo;d.controls.suppressTicks=true;const s=d.demoStatus();s.state='SUCCEEDED';s.task={...s.task,backend:'lightnav',task_mode:'imagegoal',outcome:{reason:'go2_imagegoal_model_stop',arrival_independently_validated:false}};d.emit([{topic:'/imagegoal/shadow/status',message:{data:JSON.stringify(s)}}]);});
  assert.equal(await page.locator('.ig-state').innerText(),'Policy stopped · arrival unconfirmed');assert.equal((await page.evaluate(()=>globalThis.imagegoalDemo.published)).length,0);await page.close();
});


test('English redesign follows theme and keeps priority robot Stop visible while scrolling',async()=>{
  for(const theme of ['dark','light']){
    const page=await browser.newPage({viewport:{width:1280,height:800}});
    await page.goto(base+'?fixture=native&theme='+theme);
    await page.getByTestId('capture').click();await page.waitForFunction(()=>!document.querySelector('[data-testid="native_start"]').disabled);
    assert.equal(await page.locator('.ig-app').getAttribute('data-theme'),theme);assert.match(await page.locator('#demo-notice').innerText(),/Simulated data.*No robot connection/);
    assert.equal(await page.locator('.ig-app').getAttribute('lang'),'en');
    assert.equal(/\p{Script=Han}/u.test(await page.locator('.ig-app').innerText()),false);
    assert.equal(await page.locator('header [data-testid="native_stop"]').count(),1);
    const stopBox=await page.getByTestId('native_stop').boundingBox();assert.ok(stopBox.y>=0&&stopBox.y+stopBox.height<800);
    await page.screenshot({path:join(directory,'preview-desktop-'+theme+'.png'),fullPage:true});
    if(theme==='dark'){await page.getByTestId('tracking').click();await page.waitForTimeout(250);await page.locator('.ig-app').evaluate(el=>el.scrollTop=0);await page.screenshot({path:join(directory,'preview-tracking-ready.png'),fullPage:true});}
    await page.locator('.ig-app').evaluate(el=>el.scrollTop=el.scrollHeight);await page.waitForTimeout(80);
    const scrolled=await page.getByTestId('native_stop').boundingBox();assert.ok(scrolled.y>=0&&scrolled.y+scrolled.height<800);
    await page.close();
  }
  const page=await browser.newPage({viewport:{width:390,height:844}});await page.goto(base+'?fixture=native&theme=light');
  await page.getByTestId('tracking').click();await page.waitForTimeout(250);await page.locator('.ig-app').evaluate(el=>el.scrollTop=0);
  assert.equal(await page.evaluate(()=>document.querySelector('.ig-app').scrollWidth>document.querySelector('.ig-app').clientWidth),false);
  await page.screenshot({path:join(directory,'preview-mobile-light.png'),fullPage:true});
  await page.locator('.ig-app').evaluate(el=>el.scrollTop=el.scrollHeight);await page.waitForTimeout(80);
  const stop=await page.getByTestId('native_stop').boundingBox();assert.ok(stop.y>=0&&stop.y+stop.height<844);
  await page.setViewportSize({width:320,height:720});await page.waitForTimeout(100);assert.equal(await page.evaluate(()=>document.querySelector('.ig-app').scrollWidth>document.querySelector('.ig-app').clientWidth),false);
  await page.close();
});
