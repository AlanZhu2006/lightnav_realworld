#!/usr/bin/env node
/** Read-only RGB throughput measurement: only subscribes; no client advertisement or task commands. */
import { writeFile } from 'node:fs/promises';
import { performance } from 'node:perf_hooks';
import { createHash } from 'node:crypto';
import { decodeImage, decodeString } from './probe_foxglove.mjs';
const options = { url: 'ws://127.0.0.1:18766', seconds: 10, report: '/private/tmp/foxglove-video-baseline.json' };
for (let i = 2; i < process.argv.length; i++) {
  const argument = process.argv[i];
  if (!['--url','--seconds','--report'].includes(argument)) throw new Error(`Unknown argument ${argument}`);
  if (!process.argv[i+1]) throw new Error(`Missing value for ${argument}`);
  options[argument.slice(2)] = argument === '--seconds' ? Number(process.argv[++i]) : process.argv[++i];
}
if (!Number.isFinite(options.seconds) || options.seconds < 3 || options.seconds > 60) throw new Error('seconds must be 3..60');
const rgbTopic = '/imagegoal/shadow/rgb/compressed';
const statusTopic = '/imagegoal/shadow/status';
const bridgeTopic = '/imagegoal/shadow/bridge_status';
const subscriptions = new Map();
const samples = [];
const statuses = [];
const bridge = [];
const protocolStatus = [];
const topicBytes = new Map();
const started = performance.now();
let measureStart = null;
let stopAt = Infinity;
let failure = null;
let server = null;
let nextId = 1;
let timer;
let closeExpected = false;
let finish;
const finished = new Promise((resolve) => { finish = resolve; });
const ws = new WebSocket(options.url, ['foxglove.sdk.v1']);
ws.binaryType = 'arraybuffer';
function send(value) { ws.send(JSON.stringify(value)); }
function summary(values) {
  if (!values.length) return null;
  const sorted = [...values].sort((a,b)=>a-b);
  const quantile=(q)=>sorted[Math.round((sorted.length-1)*q)];
  return { count: values.length, min: sorted[0], median: quantile(.5), p90: quantile(.9), p95: quantile(.95), max: sorted.at(-1), mean: values.reduce((a,b)=>a+b,0)/values.length };
}
function stop(error) {
  if (closeExpected) return;
  failure ??= error;
  closeExpected = true;
  clearTimeout(timer);
  if (ws.readyState === WebSocket.OPEN) {
    send({ op:'unsubscribe', subscriptionIds:[...subscriptions.keys()] });
    ws.close();
  }
  finish();
}
ws.addEventListener('message', ({ data }) => {
  try {
    if (closeExpected) return;
    if (typeof data === 'string') {
      const message = JSON.parse(data);
      if (message.op === 'serverInfo') server = message;
      if (message.op === 'status') { protocolStatus.push(message); if(message.level>=2) stop(new Error(message.message)); }
      if (message.op === 'advertise') for (const channel of message.channels) {
        if (![rgbTopic,statusTopic,bridgeTopic].includes(channel.topic) || [...subscriptions.values()].some((value)=>value.id===channel.id)) continue;
        if(channel.encoding!=='cdr') throw new Error(`Unexpected ${channel.encoding} encoding on ${channel.topic}`);
        const id=nextId++;
        subscriptions.set(id,channel);
        send({ op:'subscribe', subscriptions:[{id,channelId:channel.id}] });
      }
      return;
    }
    const receivedMonotonic = performance.now();
    const receivedWall = Date.now();
    const bytes=Buffer.from(data);
    if(bytes[0]!==1) return;
    if(bytes.length<13) throw new Error('Truncated message frame');
    const channel=subscriptions.get(bytes.readUInt32LE(1));
    if(!channel) return;
    const payload=bytes.subarray(13);
    if(channel.topic===rgbTopic) {
      const image=decodeImage(payload);
      if(measureStart===null) { measureStart=receivedMonotonic; stopAt=measureStart+options.seconds*1000; timer=setTimeout(()=>stop(),options.seconds*1000); }
      if(receivedMonotonic>stopAt) return;
      samples.push({ arrival_relative_ms:receivedMonotonic-measureStart, arrival_wall_ms:receivedWall, header_timestamp_ns:image.timestamp_ns, frame_id:image.frame_id, format:image.format, jpeg_bytes:image.jpeg_bytes, jpeg_sha256:createHash('sha256').update(image.jpeg).digest('hex'), bridge_log_timestamp_ns:bytes.readBigUInt64LE(5).toString(), header_to_local_receive_clock_difference_ms:receivedWall-Number(BigInt(image.timestamp_ns)/1000000n) });
    } else if(channel.topic===statusTopic) {
      const status=JSON.parse(decodeString(payload));
      statuses.push({ arrival_relative_ms:measureStart===null?null:receivedMonotonic-measureStart, runtime_timestamp_ns:status.timestamp_ns, session_id:status.session_id, state:status.state, rgb_age_s:status.sensors?.rgb?.age_s, rgb_capture_timestamp_ns:status.sensors?.rgb?.capture_timestamp_ns, rgb_frame_id:status.sensors?.rgb?.frame_id, rgb_timestamp_source:status.sensors?.rgb?.timestamp_source, rgb_synthetic:status.sensors?.rgb?.synthetic, hardware_transport:status.hardware_transport, proposal_published_to_robot:status.proposal?.published_to_robot });
    } else bridge.push(JSON.parse(decodeString(payload)));
    if (measureStart !== null && receivedMonotonic <= stopAt) {
      const counter = topicBytes.get(channel.topic) || { messages: 0, cdr_bytes: 0, foxglove_payload_bytes: 0 };
      counter.messages++; counter.cdr_bytes += payload.length; counter.foxglove_payload_bytes += bytes.length;
      topicBytes.set(channel.topic, counter);
    }
  } catch(error) { stop(error); }
});
ws.addEventListener('error',()=>stop(new Error('WebSocket failed')));
ws.addEventListener('close',({code,reason})=>{ if(!closeExpected) stop(new Error(`WebSocket closed ${code} ${reason}`)); });
const startupTimeout=setTimeout(()=>{if(measureStart===null)stop(new Error('No RGB received within 15 seconds'));},15000);
await finished;
clearTimeout(startupTimeout);
const arrivalIntervals=samples.slice(1).map((item,index)=>item.arrival_relative_ms-samples[index].arrival_relative_ms);
const headerIntervals=samples.slice(1).map((item,index)=>Number(BigInt(item.header_timestamp_ns)-BigInt(samples[index].header_timestamp_ns))/1e6);
const spanMs=samples.length>1?samples.at(-1).arrival_relative_ms-samples[0].arrival_relative_ms:0;
const displayTime=samples.some((item)=>item.frame_id.includes('display_time'));
const report={ schema_version:'imagegoal.foxglove_video_measurement.v1', measured_at:new Date().toISOString(), mode:'read_only', url:options.url, requested_duration_s:options.seconds, elapsed_total_s:(performance.now()-started)/1000, ok:!failure&&samples.length>=2, error:failure?.message, server:{protocol:ws.protocol,name:server?.name}, topic:rgbTopic, frames:samples.length, fps_over_fixed_window:samples.length/options.seconds, fps_between_first_last:spanMs>0?(samples.length-1)/(spanMs/1000):null, interframe_arrival_ms:summary(arrivalIntervals), interframe_header_ms:summary(headerIntervals), jpeg_bytes:summary(samples.map((item)=>item.jpeg_bytes)), unique_jpeg_hashes:new Set(samples.map((item)=>item.jpeg_sha256)).size, source_latency_observable:!displayTime&&samples.length>0, timestamp_note:displayTime?'Header is bridge display time, not camera exposure time. Header-to-receive differences are cross-host clock differences and MUST NOT be claimed as camera source latency.':'Header preserves source stamp; header-to-receive clock differences still require synchronized clocks for latency interpretation.', header_to_local_receive_clock_difference_ms:summary(samples.map((item)=>item.header_to_local_receive_clock_difference_ms)), runtime_rgb_age_s:summary(statuses.map((item)=>item.rgb_age_s).filter(Number.isFinite)), runtime_status_count:statuses.length, throughput_by_topic:Object.fromEntries([...topicBytes].map(([topic,value])=>[topic,{...value,bytes_per_second:value.foxglove_payload_bytes/options.seconds,megabits_per_second:value.foxglove_payload_bytes*8/options.seconds/1e6}])), header_clock_difference_growth_ms:samples.length>1?samples.at(-1).header_to_local_receive_clock_difference_ms-samples[0].header_to_local_receive_clock_difference_ms:null, bridge_status_latest:bridge.at(-1), protocol_status:protocolStatus, samples, runtime_status_samples:statuses };
await writeFile(options.report,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({report:options.report,ok:report.ok,frames:report.frames,fps:report.fps_between_first_last,arrival_ms:report.interframe_arrival_ms,header_ms:report.interframe_header_ms,source_latency_observable:report.source_latency_observable,frame_id:samples[0]?.frame_id,runtime_rgb_age_s:report.runtime_rgb_age_s,error:report.error},null,2));
process.exitCode=report.ok?0:1;
// A saturated incoming stream can postpone the WebSocket closing handshake.
// Reports are already flushed; bound this measurement process's lifetime.
setTimeout(()=>process.exit(process.exitCode),1000).unref();
