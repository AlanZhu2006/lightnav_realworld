#!/usr/bin/env node
/**
 * Real Foxglove SDK v1 / ROS 2 integration probe (Node >= 22, no dependencies).
 * Default: subscribe only. --exercise-shadow explicitly sends bounded shadow
 * commands; it refuses to replace an active task and always attempts Stop.
 *
 * node deployment/operator/tests/probe_foxglove.mjs --url ws://127.0.0.1:18766 --report /tmp/foxglove.json
 * node deployment/operator/tests/probe_foxglove.mjs --exercise-shadow [--lightnav] --report /tmp/foxglove-shadow.json
 * node deployment/operator/tests/probe_foxglove.mjs --self-test
 *
 * Protocol verified against official current SDK sources:
 * https://github.com/foxglove/foxglove-sdk/blob/main/rust/foxglove/src/protocol/common/client/advertise.rs
 * https://github.com/foxglove/foxglove-sdk/blob/main/rust/foxglove/src/protocol/common/client/message_data.rs
 * https://github.com/foxglove/foxglove-sdk/blob/main/rust/foxglove/src/protocol/v1/server/message_data.rs
 * https://docs.foxglove.dev/docs/getting-started/custom/custom-schema-encodings
 * ros2msg schemas are UTF-8 text, not base64. Server op1 includes a timestamp;
 * client op1 has only a channel ID before the CDR payload.
 */
import assert from 'node:assert/strict';
import { writeFile, mkdir } from 'node:fs/promises';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { createHash, randomUUID } from 'node:crypto';

const TOPICS = {
  status: '/imagegoal/shadow/status',
  bridge: '/imagegoal/shadow/bridge_status',
  rgb: '/imagegoal/shadow/rgb/compressed',
  nativeRgb: '/go2/front_camera/image_raw/compressed',
  goal: '/imagegoal/shadow/goal/compressed',
  result: '/imagegoal/operator/result',
  localPath: '/imagegoal/shadow/local_path',
};
const TERMINAL = new Set(['IDLE', 'CANCELLED', 'SUCCEEDED', 'TIMED_OUT', 'FAILED']);
const IMAGE_TOPICS = new Set(['rgb', 'nativeRgb', 'goal']);
const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
const sha256 = (bytes) => createHash('sha256').update(bytes).digest('hex');

export function resultStatus(result) { return result?.schema_version === 'imagegoal.runtime.v1' ? result : result?.status; }

export function frozenSelection(status) {
  const selection = status?.goal_selection;
  assert.equal(selection?.schema_version, 'imagegoal.goal_selection.v1', 'missing canonical goal selection');
  assert.equal(selection.session_id, status.session_id, 'selection belongs to a different runtime');
  assert.equal(selection.mode, 'frozen', 'goal must be frozen');
  assert.equal(selection.ready, true, 'goal must be ready');
  assert(typeof selection.selection_id === 'string' && selection.selection_id, 'missing selection ID');
  assert(/^[0-9a-f]{64}$/.test(selection.sha256), 'invalid frozen JPEG SHA256');
  return selection;
}

export function commandPayload(action, status, payload, session) {
  assert.equal(status?.session_id, session, 'runtime session changed');
  if (action === 'start') {
    const selection = frozenSelection(status);
    assert(!Object.hasOwn(payload, 'goal'), 'start must reference the frozen selection, not import a goal');
    assert.equal(payload.selection_id, selection.selection_id, 'start selection differs from the frozen goal');
    assert.equal(payload.execution_mode, 'shadow', 'probe can only start shadow tasks');
  }
  return { ...payload, expected_session_id: session, ...(action === 'cancel' ? {} : { expected_epoch: status.epoch }) };
}

export function ownershipGuard(status, { session, taskId, selectionId }) {
  shadowGuard(status);
  assert.equal(status.session_id, session, 'runtime session changed');
  if (taskId) assert.equal(status.task?.task_id, taskId, 'owned task changed; refusing to alter another task');
  else assert(TERMINAL.has(status.state), 'refusing to alter an active task whose ownership is unconfirmed');
  if (selectionId) assert.equal(status.goal_selection?.selection_id, selectionId, 'owned goal selection changed; refusing to alter another selection');
}

export class CdrReader {
  constructor(bytes) {
    this.bytes = Buffer.from(bytes);
    assert(this.bytes.length >= 4, 'truncated CDR encapsulation');
    const representation = this.bytes.readUInt16BE(0);
    assert(representation === 0 || representation === 1, `unsupported CDR representation ${representation}`);
    this.little = representation === 1;
    this.position = 4;
  }
  align(n) { this.position += (n - (this.position - 4) % n) % n; }
  take(n) {
    assert(Number.isSafeInteger(n) && n >= 0 && this.position + n <= this.bytes.length, 'truncated CDR');
    const start = this.position;
    this.position += n;
    return this.bytes.subarray(start, this.position);
  }
  uint32() {
    this.align(4);
    const value = this.take(4);
    return this.little ? value.readUInt32LE(0) : value.readUInt32BE(0);
  }
  string() {
    const n = this.uint32();
    assert(n > 0 && n <= 8 * 1024 * 1024, 'invalid CDR string length');
    const value = this.take(n);
    assert.equal(value[n - 1], 0, 'CDR string lacks NUL');
    return value.subarray(0, n - 1).toString('utf8');
  }
  header() {
    const sec = this.uint32();
    const nanosec = this.uint32();
    assert(nanosec < 1e9, 'invalid ROS timestamp');
    return { timestamp_ns: (BigInt(sec) * 1000000000n + BigInt(nanosec)).toString(), frame_id: this.string() };
  }
}

export function encodeString(value) {
  const text = Buffer.from(value, 'utf8');
  const bytes = Buffer.alloc(9 + text.length);
  bytes[1] = 1; // CDR little-endian encapsulation, options=0
  bytes.writeUInt32LE(text.length + 1, 4);
  text.copy(bytes, 8);
  return bytes;
}

export function decodeString(value) { return new CdrReader(value).string(); }
export function decodeImage(value) {
  const reader = new CdrReader(value);
  const header = reader.header();
  const format = reader.string();
  const data = reader.take(reader.uint32());
  assert(data.length >= 2 && data[0] === 0xff && data[1] === 0xd8, 'compressed preview is not JPEG');
  return { ...header, format, jpeg_bytes: data.length, jpeg: data };
}
export function decodePath(value) {
  const reader = new CdrReader(value);
  const header = reader.header();
  const poses = reader.uint32();
  assert(poses <= 10000, 'implausible path length');
  for (let i = 0; i < poses; i++) {
    reader.header();
    reader.align(8);
    const fields = reader.take(7 * 8);
    for (let j = 0; j < 7; j++) assert(Number.isFinite(reader.little ? fields.readDoubleLE(j * 8) : fields.readDoubleBE(j * 8)), 'nonfinite path pose');
  }
  return { ...header, poses };
}

export function shadowGuard(status) {
  assert.equal(status?.schema_version, 'imagegoal.runtime.v1', 'unexpected runtime schema');
  assert.equal(status.hardware_transport, 'absent', 'hardware transport must be absent');
  assert.equal(status.not_for_motion, true, 'runtime must report not_for_motion');
  assert.equal(status.proposal?.published_to_robot, false, 'proposal must not have been published to robot');
  assert.equal(status.proposal?.not_for_motion, true, 'proposal must be diagnostic');
  assert.equal(status.synthetic_feedback, false, 'this probe requires real source feedback');
  assert.equal(status.execution_envelope?.eligible_for_robot_tracking, false, 'robot execution envelope must not be eligible');
  assert(!status.task || status.task.execution_mode === 'shadow', 'refusing an operator_run task');
  assert(Number.isInteger(status.epoch) && status.epoch >= 0, 'invalid runtime epoch');
  assert(typeof status.session_id === 'string' && status.session_id, 'missing runtime session');
}

class Probe {
  constructor(url, report) {
    this.report = report;
    this.channels = new Map();
    this.subscriptions = new Map();
    this.latest = {};
    this.count = {};
    this.goalFrames = [];
    this.pending = new Map();
    this.statusVersion = 0;
    this.error = null;
    this.exercise = false;
    this.closed = false;
    this.ws = new WebSocket(url, ['foxglove.sdk.v1']);
    this.ws.binaryType = 'arraybuffer';
    this.ws.addEventListener('message', (event) => {
      try { this.message(event.data); } catch (error) { this.error ??= error; }
    });
    this.ws.addEventListener('error', () => { this.error ??= new Error('Foxglove WebSocket connection failed'); });
    this.ws.addEventListener('close', (event) => {
      this.closed = true;
      if (!this.closing) this.error ??= new Error(`Foxglove closed (${event.code}) ${event.reason}`);
    });
  }
  json(value) { assert.equal(this.ws.readyState, WebSocket.OPEN, 'Foxglove socket is not open'); this.ws.send(JSON.stringify(value)); }
  subscribe(channel) {
    const name = Object.keys(TOPICS).find((key) => TOPICS[key] === channel.topic);
    if (!name || [...this.subscriptions.values()].some((item) => item.id === channel.id)) return;
    assert.equal(channel.encoding, 'cdr', `${channel.topic} is not CDR`);
    const expected = IMAGE_TOPICS.has(name) ? 'sensor_msgs/msg/CompressedImage' : name === 'localPath' ? 'nav_msgs/msg/Path' : 'std_msgs/msg/String';
    assert.equal(channel.schemaName.replace(/\/(msg)\//, '/'), expected.replace('/msg/', '/'), `${channel.topic} has unexpected schema`);
    const id = this.subscriptions.size + 1;
    this.subscriptions.set(id, { ...channel, name });
    this.json({ op: 'subscribe', subscriptions: [{ id, channelId: channel.id }] });
  }
  message(data) {
    if (typeof data === 'string') {
      const value = JSON.parse(data);
      if (value.op === 'serverInfo') this.server = value;
      if (value.op === 'advertise') for (const channel of value.channels) {
        this.channels.set(channel.id, channel);
        this.subscribe(channel);
      }
      if (value.op === 'status') {
        this.report.protocol_status.push({ level: value.level, message: value.message });
        if (value.level >= 2) throw new Error(`Foxglove protocol error: ${value.message}`);
      }
      return;
    }
    const frame = Buffer.from(data);
    assert(frame.length > 0, 'empty binary frame');
    if (frame[0] !== 1) return;
    assert(frame.length >= 13, 'truncated server data frame');
    const subscription = this.subscriptions.get(frame.readUInt32LE(1));
    if (!subscription) return;
    const name = subscription.name;
    const payload = frame.subarray(13);
    const receivedAt = Date.now();
    const decoded = IMAGE_TOPICS.has(name) ? decodeImage(payload) : name === 'localPath' ? decodePath(payload) : JSON.parse(decodeString(payload));
    this.count[name] = (this.count[name] || 0) + 1;
    this.latest[name] = { value: decoded, received_at_ms: receivedAt, log_timestamp_ns: frame.readBigUInt64LE(5).toString() };
    if (name === 'goal') {
      this.goalFrames.push({ frame_id: decoded.frame_id, sha256: sha256(decoded.jpeg), received_at_ms: receivedAt, count: this.count.goal });
      this.goalFrames.splice(0, Math.max(0, this.goalFrames.length - 128));
    }
    if (name === 'status') {
      assert.equal(decoded.schema_version, 'imagegoal.runtime.v1');
      this.statusVersion++;
      if (this.exercise) shadowGuard(decoded);
    }
    if (name === 'result') {
      const pending = this.pending.get(decoded.request_id);
      if (pending) {
        clearTimeout(pending.timer);
        this.pending.delete(decoded.request_id);
        decoded.ok ? pending.resolve(decoded.result) : pending.reject(new Error(`${decoded.action}: ${decoded.error}`));
      }
    }
  }
  async waitFor(predicate, label, timeoutMs = 10000) {
    const started = Date.now();
    while (Date.now() - started < timeoutMs) {
      if (this.error) throw this.error;
      const value = predicate();
      if (value) return value;
      await delay(40);
    }
    throw new Error(`timeout waiting for ${label}`);
  }
  status() { return this.latest.status?.value; }
  async freshStatus(afterVersion = this.statusVersion, timeoutMs = 4000) {
    return this.waitFor(() => this.statusVersion > afterVersion && (!this.ackStatus || this.status()?.session_id !== this.ackStatus.session_id || this.status()?.epoch >= this.ackStatus.epoch) && this.status(), 'fresh runtime status', timeoutMs);
  }
  async advertise() {
    this.json({ op: 'advertise', channels: [{ id: 1, topic: '/imagegoal/operator/command', encoding: 'cdr', schemaName: 'std_msgs/msg/String', schemaEncoding: 'ros2msg', schema: 'string data\n' }] });
    // DDS discovery can lag the WebSocket advertisement; subscribe/status remain
    // live during this bounded settling interval. No task is active at this point.
    await delay(500);
    if (this.error) throw this.error;
  }
  async command(action, payload = {}, { cleanup = false } = {}) {
    assert(this.exercise && this.advertised, 'command publishing requires --exercise-shadow');
    const status = cleanup ? this.status() : await this.freshStatus();
    ownershipGuard(status, this);
    const request_id = `probe-${randomUUID()}`;
    const body = { request_id, session_id: this.session, action, payload: commandPayload(action, status, payload, this.session) };
    const started = Date.now();
    const result = new Promise((resolve, reject) => {
      const timer = setTimeout(() => { this.pending.delete(request_id); reject(new Error(`${action}: no command acknowledgement`)); }, cleanup ? 4000 : 8500);
      this.pending.set(request_id, { resolve, reject, timer });
    });
    const cdr = encodeString(JSON.stringify(body));
    const frame = Buffer.alloc(5 + cdr.length);
    frame[0] = 1;
    frame.writeUInt32LE(1, 1);
    cdr.copy(frame, 5);
    this.ws.send(frame);
    try {
      const value = await result;
      const acknowledged = resultStatus(value);
      this.report.commands.push({ action, ok: true, round_trip_ms: Date.now() - started, epoch: acknowledged?.epoch, state: acknowledged?.state,
        selection_id: acknowledged?.goal_selection?.selection_id, task_selection_id: acknowledged?.task?.selection_id });
      if (acknowledged) {
        shadowGuard(acknowledged);
        assert.equal(acknowledged.session_id, this.session);
        // Command acknowledgement can be newer than the status subscription.
        // The next action still waits for a fresh status before choosing epoch.
        this.ackStatus = acknowledged;
      }
      return value;
    } catch (error) {
      this.report.commands.push({ action, ok: false, error: error.message, round_trip_ms: Date.now() - started });
      throw error;
    }
  }
  async stopOwned() {
    if (!this.startAttempted || this.closed) return;
    const status = this.status();
    if (status?.session_id !== this.session) throw new Error('cannot Stop: runtime session changed');
    if (!this.taskId) {
      assert(TERMINAL.has(status.state), 'cannot Stop: start ownership was not acknowledged; bounded shadow task must expire independently');
      this.startAttempted = false;
      return status;
    }
    if (this.taskId && status.task?.task_id !== this.taskId) throw new Error('cannot Stop: a different task is now active');
    const stopped = await this.command('cancel', {}, { cleanup: true });
    assert(TERMINAL.has(stopped.state), 'cancel did not return a terminal state');
    assert.equal(stopped.proposal.vx_m_s, 0);
    assert.equal(stopped.proposal.vy_m_s, 0);
    assert.equal(stopped.proposal.wz_rad_s, 0);
    await this.waitFor(() => this.status()?.epoch >= stopped.epoch && this.latest.localPath?.value.poses === 0, 'cancel status and cleared local path', 4000);
    this.report.cleanup = { ok: true, state: stopped.state, epoch: stopped.epoch, local_path_cleared: true };
    this.startAttempted = false;
    this.taskId = null;
    return stopped;
  }
  async snapshot(directory, status, backend) {
    await mkdir(directory, { recursive: true });
    const files = { status: join(directory, `${backend}-status.json`) };
    await writeFile(files.status, JSON.stringify(status, null, 2) + '\n');
    const imageHeaders = {};
    for (const name of ['rgb', 'nativeRgb', 'goal']) {
      const image = this.latest[name]?.value;
      if (image?.jpeg) {
        files[name] = join(directory, `${backend}-${name}.jpg`);
        await writeFile(files[name], image.jpeg);
        imageHeaders[name] = { timestamp_ns: image.timestamp_ns, frame_id: image.frame_id, format: image.format, jpeg_bytes: image.jpeg_bytes };
      }
    }
    const manifest = { recorded_snapshot: true, captured_at: new Date().toISOString(), runtime_timestamp_ns: status.timestamp_ns, session_id: status.session_id, task_id: status.task?.task_id, backend, source_timestamps_preserved: true, image_headers: imageHeaders, local_path: this.latest.localPath?.value, files };
    await writeFile(join(directory, `${backend}-snapshot.json`), JSON.stringify(manifest, null, 2) + '\n');
    this.report.snapshots ??= [];
    this.report.snapshots.push(manifest);
  }
  async exerciseBackend(backend, snapshotDirectory) {
    const initial = await this.freshStatus();
    shadowGuard(initial);
    assert(TERMINAL.has(initial.state), `refusing to replace active ${initial.state} task`);
    this.session = initial.session_id;
    this.selectionId = null;
    const captured = await this.command('capture_goal');
    const captureStatus = resultStatus(captured);
    const selection = frozenSelection(captureStatus);
    this.selectionId = selection.selection_id;
    assert.equal(selection.source, 'camera');
    assert.equal(captured.selection?.selection_id, selection.selection_id);
    assert(TERMINAL.has(captureStatus.state), 'capture unexpectedly started a task');
    assert.equal(captureStatus.task?.task_id, initial.task?.task_id, 'capture created a navigation task');
    assert.equal(captureStatus.metrics.inference_started, initial.metrics.inference_started, 'capture started model inference');
    assert(captureStatus.epoch > initial.epoch, 'capture did not establish a selection barrier');
    const raw = Buffer.from(captured.goal?.rgb_jpeg_base64 ?? '', 'base64');
    assert(raw.length > 2 && raw[0] === 0xff && raw[1] === 0xd8, 'capture_goal did not return raw JPEG');
    assert.equal(sha256(raw), selection.sha256, 'capture JPEG differs from canonical frozen goal');
    const expectedFrame = `imagegoal_frozen_goal_${selection.selection_id}`;
    await this.waitFor(() => this.latest.goal?.value.frame_id === expectedFrame, 'captured goal image with selection identity', 6500);
    assert.equal(sha256(this.latest.goal.value.jpeg), selection.sha256);
    const goalCount = this.count.goal;
    const rgbCount = this.count.rgb;
    const nativeCount = this.count.nativeRgb;
    const rgbStamp = this.latest.rgb.value.timestamp_ns;
    const nativeStamp = this.latest.nativeRgb.value.timestamp_ns;
    // The goal is low-rate state (five-second replay), while both live image
    // streams must advance. Two distinct goal publications prove freezing at
    // the bridge boundary without requiring scene motion or robot movement.
    const stable = await this.waitFor(() => {
      const status = this.status();
      return this.count.goal > goalCount && this.count.rgb >= rgbCount + 2 && this.count.nativeRgb >= nativeCount + 2 &&
        status?.goal_selection?.selection_id === selection.selection_id && status;
    }, 'frozen goal replay while runtime and native RGB advance', 8000);
    ownershipGuard(stable, this);
    assert.equal(frozenSelection(stable).sha256, selection.sha256);
    assert.equal(stable.metrics.inference_started, captureStatus.metrics.inference_started, 'selection preview started inference');
    assert.notEqual(this.latest.rgb.value.timestamp_ns, rgbStamp, 'runtime RGB source stamp did not advance');
    assert.notEqual(this.latest.nativeRgb.value.timestamp_ns, nativeStamp, 'native camera source stamp did not advance');
    const frozenFrames = this.goalFrames.filter((frame) => frame.frame_id === expectedFrame);
    assert(frozenFrames.length >= 2, 'need two frozen-image publications');
    assert(frozenFrames.every((frame) => frame.sha256 === selection.sha256), 'frozen goal changed as live RGB advanced');
    const startedAt = Date.now();
    this.startAttempted = true;
    let outcome;
    try {
      const started = await this.command('start', { selection_id: selection.selection_id, backend, execution_mode: 'shadow', timeout_s: 30, instruction: '' });
      this.taskId = started.task.task_id;
      assert.equal(started.task.backend, backend);
      assert.equal(started.task.execution_mode, 'shadow');
      assert.equal(started.task.selection_id, selection.selection_id);
      assert.equal(started.task.goal_sha256, selection.sha256);
      assert.equal(frozenSelection(started).selection_id, selection.selection_id);
      assert(started.task.timeout_s <= 30);
      const previewStatus = await this.waitFor(() => {
        const status = this.status();
        const preview = status?.preview;
        return status?.task?.task_id === this.taskId && preview?.identity?.task_id === this.taskId && preview.epoch === status.epoch && preview.goal_version === status.task.goal_version && preview.trajectory_body_m?.length > 1 && status;
      }, `${backend} real model preview`, 20000);
      shadowGuard(previewStatus);
      assert.equal(previewStatus.sensors?.rgb?.synthetic, false, 'preview input must be measured RGB');
      await this.waitFor(() => this.latest.localPath?.value.poses > 0 && this.latest.goal?.value.frame_id === expectedFrame, `${backend} local path and frozen goal JPEG`, 3000);
      assert.equal(this.latest.localPath.value.frame_id, 'imagegoal_model_local');
      outcome = { backend, task_id: this.taskId, selection_id: selection.selection_id, goal_sha256: selection.sha256,
        capture_did_not_start: true, frozen_goal_publications: frozenFrames.length, frozen_bytes_unchanged: true,
        runtime_rgb_advanced: true, native_rgb_advanced: true, start_selection_bound: true,
        raw_goal_jpeg_bytes: raw.length, preview_points: previewStatus.preview.trajectory_body_m.length,
        preview_backend: previewStatus.preview.backend, preview_frame: previewStatus.preview.frame,
        odom_available: previewStatus.sensors?.odom != null, local_path_poses: this.latest.localPath.value.poses,
        gates: previewStatus.gates, preview_after_ms: Date.now() - startedAt, navigation_success_validated: false };
      assert.equal(previewStatus.preview.backend, backend);
      if (snapshotDirectory) await this.snapshot(snapshotDirectory, previewStatus, backend);
      // Exercise controls before the independently enforced 30-second deadline.
      assert(Date.now() - startedAt < 26000, 'too little time remains for shadow control exercise');
      const paused = await this.command('pause');
      assert.equal(paused.state, 'PAUSED');
      assert.equal(paused.task.paused, true);
      await this.waitFor(() => this.status()?.epoch >= paused.epoch && this.latest.localPath?.value.poses === 0, 'pause cleared local path', 2000);
      const resumed = await this.command('resume');
      assert.equal(resumed.task.paused, false);
      outcome.controls = { paused: true, resumed: true };
    } finally {
      const stopped = await this.stopOwned();
      if (stopped && outcome) {
        assert.equal(frozenSelection(stopped).selection_id, selection.selection_id, 'Stop discarded the frozen goal');
        assert.equal(stopped.goal_selection.sha256, selection.sha256);
        outcome.stop_preserved_selection = true;
      }
    }
    const cleared = await this.command('clear_goal');
    assert.equal(cleared.goal_selection?.mode, 'live');
    assert.equal(cleared.goal_selection?.ready, false);
    assert.equal(cleared.goal_selection?.selection_id, null);
    assert(TERMINAL.has(cleared.state), 'clear left an active task');
    this.selectionId = null;
    outcome.clear_returned_live = true;
    const imported = await this.command('import_goal', { goal: { rgb_jpeg_base64: raw.toString('base64') } });
    const importStatus = resultStatus(imported);
    const importedSelection = frozenSelection(importStatus);
    this.selectionId = importedSelection.selection_id;
    assert.equal(importedSelection.source, 'import');
    assert.notEqual(importedSelection.selection_id, selection.selection_id);
    assert.equal(importedSelection.sha256, selection.sha256);
    assert.equal(sha256(Buffer.from(imported.goal?.rgb_jpeg_base64 ?? '', 'base64')), selection.sha256);
    assert(TERMINAL.has(importStatus.state), 'import unexpectedly started a task');
    assert.equal(importStatus.task?.task_id, cleared.task?.task_id, 'import created a task');
    assert.equal(importStatus.metrics.inference_started, cleared.metrics.inference_started, 'import started inference');
    await this.waitFor(() => this.latest.goal?.value.frame_id === `imagegoal_frozen_goal_${importedSelection.selection_id}`, 'imported goal publication', 6500);
    assert.equal(sha256(this.latest.goal.value.jpeg), selection.sha256);
    outcome.import = { selection_id: importedSelection.selection_id, source: importedSelection.source, exact_jpeg: true, did_not_start: true };
    const final = await this.command('clear_goal');
    assert.equal(final.goal_selection?.mode, 'live');
    assert.equal(final.goal_selection?.ready, false);
    this.selectionId = null;
    outcome.final_live = true;
    this.report.exercises.push(outcome);
  }
  close() {
    this.closing = true;
    for (const pending of this.pending.values()) {
      clearTimeout(pending.timer);
      pending.reject(new Error('probe closed'));
    }
    this.pending.clear();
    if (this.ws.readyState === WebSocket.OPEN) {
      if (this.advertised) this.json({ op: 'unadvertise', channelIds: [1] });
      this.json({ op: 'unsubscribe', subscriptionIds: [...this.subscriptions.keys()] });
      this.ws.close();
    }
  }
}

function options(argv) {
  const result = { url: 'ws://127.0.0.1:18766', report: null, snapshotDir: null, exercise: false, lightnav: false, selfTest: false };
  for (let i = 0; i < argv.length; i++) {
    const argument = argv[i];
    if (argument === '--url' || argument === '--report' || argument === '--snapshot-dir') { assert(argv[i + 1], `${argument} requires a value`); result[argument === '--snapshot-dir' ? 'snapshotDir' : argument.slice(2)] = argv[++i]; }
    else if (argument === '--exercise-shadow') result.exercise = true;
    else if (argument === '--lightnav') result.lightnav = true;
    else if (argument === '--self-test') result.selfTest = true;
    else if (argument === '--help') { console.log('Usage: node probe_foxglove.mjs [--url ws://127.0.0.1:18766] [--report path] [--snapshot-dir path] [--exercise-shadow [--lightnav]] [--self-test]'); return null; }
    else throw new Error(`unknown argument ${argument}`);
  }
  const url = new URL(result.url);
  assert(['ws:', 'wss:'].includes(url.protocol), 'URL must use ws:// or wss://');
  assert(!result.lightnav || result.exercise, '--lightnav requires --exercise-shadow');
  return result;
}

export async function selfTest() {
  for (const text of ['', 'hello', '真实图像🦊', '{"request_id":"probe-1"}']) assert.equal(decodeString(encodeString(text)), text);
  const malformed = encodeString('test'); malformed[malformed.length - 1] = 1;
  assert.throws(() => decodeString(malformed));
  assert.throws(() => decodeString(Buffer.from([0, 1, 0, 0, 8, 0, 0, 0])));
  const raw = Buffer.from([0xff, 0xd8, 1, 2, 0xff, 0xd9]);
  const selection = { schema_version: 'imagegoal.goal_selection.v1', session_id: 'session', mode: 'frozen', ready: true, selection_id: 'selection-1', sha256: sha256(raw) };
  const status = { schema_version: 'imagegoal.runtime.v1', hardware_transport: 'absent', not_for_motion: true, proposal: { published_to_robot: false, not_for_motion: true }, synthetic_feedback: false, execution_envelope: { eligible_for_robot_tracking: false }, epoch: 1, session_id: 'session', state: 'IDLE', goal_selection: selection };
  shadowGuard(status);
  assert.throws(() => shadowGuard({ ...status, hardware_transport: 'present' }));
  assert.throws(() => shadowGuard({ ...status, task: { execution_mode: 'operator_run' } }));
  assert.throws(() => shadowGuard({ ...status, synthetic_feedback: true }));
  assert.equal(resultStatus(status), status);
  assert.equal(resultStatus({ status, selection }), status);
  assert.equal(frozenSelection(status).selection_id, 'selection-1');
  assert.throws(() => frozenSelection({ ...status, goal_selection: { ...selection, ready: false } }));
  assert.throws(() => frozenSelection({ ...status, goal_selection: { ...selection, session_id: 'old' } }));
  const capturePayload = commandPayload('capture_goal', status, {}, 'session');
  assert.deepEqual(capturePayload, { expected_session_id: 'session', expected_epoch: 1 });
  const startPayload = commandPayload('start', status, { selection_id: 'selection-1', execution_mode: 'shadow' }, 'session');
  assert(!Object.hasOwn(startPayload, 'goal'));
  assert.equal(startPayload.selection_id, selection.selection_id);
  for (const payload of [ { selection_id: 'other', execution_mode: 'shadow' }, { goal: {}, selection_id: 'selection-1', execution_mode: 'shadow' }, { selection_id: 'selection-1', execution_mode: 'operator_run' } ]) {
    assert.throws(() => commandPayload('start', status, payload, 'session'));
  }
  assert.throws(() => commandPayload('clear_goal', status, {}, 'old-session'));
  ownershipGuard(status, { session: 'session' });
  const active = { ...status, state: 'TRACKING', task: { task_id: 'own-task', execution_mode: 'shadow' } };
  ownershipGuard(active, { session: 'session', taskId: 'own-task', selectionId: 'selection-1' });
  assert.throws(() => ownershipGuard(active, { session: 'session' }));
  assert.throws(() => ownershipGuard(active, { session: 'session', taskId: 'other-task' }));
  assert.throws(() => ownershipGuard(status, { session: 'session', selectionId: 'other-selection' }));

  // A synthetic native-camera channel takes exactly the same Image decoder as
  // inference RGB. This invokes the actual advertisement/message handlers with
  // a fake socket, and never constructs WebSocket or opens a network channel.
  const fake = Object.create(Probe.prototype);
  Object.assign(fake, { channels: new Map(), subscriptions: new Map(), latest: {}, count: {}, goalFrames: [],
    pending: new Map(), report: { protocol_status: [] }, statusVersion: 0, json: (value) => fake.sent = value });
  fake.subscribe({ id: 81, topic: TOPICS.nativeRgb, encoding: 'cdr', schemaName: 'sensor_msgs/msg/CompressedImage' });
  assert.equal(fake.sent.subscriptions[0].channelId, 81);
  assert.equal([...fake.subscriptions.values()][0].name, 'nativeRgb');
  const header = encodeString('go2_front_camera_optical_frame').subarray(4);
  const format = encodeString('jpeg').subarray(4);
  const chunks = [Buffer.from([0, 1, 0, 0]), Buffer.alloc(8), header];
  chunks[1].writeUInt32LE(17, 0); chunks[1].writeUInt32LE(123, 4);
  let position = 8 + header.length;
  chunks.push(Buffer.alloc((4 - position % 4) % 4), format);
  position += (4 - position % 4) % 4 + format.length;
  chunks.push(Buffer.alloc((4 - position % 4) % 4));
  const length = Buffer.alloc(4); length.writeUInt32LE(raw.length);
  chunks.push(length, raw);
  const imageCdr = Buffer.concat(chunks);
  const wire = Buffer.alloc(13 + imageCdr.length);
  wire[0] = 1; wire.writeUInt32LE(1, 1); wire.writeBigUInt64LE(42n, 5); imageCdr.copy(wire, 13);
  fake.message(wire);
  assert.equal(fake.count.nativeRgb, 1);
  assert.equal(fake.latest.nativeRgb.value.frame_id, 'go2_front_camera_optical_frame');
  assert.equal(fake.latest.nativeRgb.value.timestamp_ns, '17000000123');
  assert.deepEqual(fake.latest.nativeRgb.value.jpeg, raw);
  assert.throws(() => fake.subscribe({ id: 82, topic: TOPICS.rgb, encoding: 'cdr', schemaName: 'std_msgs/msg/String' }));

  // Read-only behavior remains fail closed even if a caller invokes command().
  fake.exercise = false;
  await assert.rejects(() => fake.command('capture_goal'), /exercise-shadow/);
  const unknown = Object.create(Probe.prototype);
  Object.assign(unknown, { startAttempted: true, closed: false, session: 'session', taskId: null, status: () => active });
  await assert.rejects(() => unknown.stopOwned(), /ownership was not acknowledged/);

  // Run the complete exercise state machine against an in-memory command
  // transport. It checks real command serialization and acknowledgement
  // normalization without opening a socket, model, camera or robot.
  const flow = Object.create(Probe.prototype);
  const fixtureStatus = { ...status, epoch: 0, task: null, metrics: { inference_started: 0 },
    sensors: { rgb: { synthetic: false }, odom: null },
    proposal: { ...status.proposal, vx_m_s: 0, vy_m_s: 0, wz_rad_s: 0 },
    goal_selection: { ...selection, mode: 'live', ready: false, selection_id: null, sha256: null } };
  Object.assign(flow, { report: { commands: [], exercises: [] }, exercise: true, advertised: true, closed: false,
    session: 'session', taskId: null, selectionId: null, pending: new Map(), goalFrames: [], count: { rgb: 1, nativeRgb: 1, goal: 0 },
    latest: { status: { value: fixtureStatus }, rgb: { value: { timestamp_ns: '1' } },
      nativeRgb: { value: { timestamp_ns: '1' } }, localPath: { value: { poses: 0 } } } });
  let selected = 0;
  const actions = [];
  function fixtureGoal(nextSelection) {
    const frameId = `imagegoal_frozen_goal_${nextSelection.selection_id}`;
    flow.latest.goal = { value: { frame_id: frameId, jpeg: raw } };
    flow.count.goal++;
    flow.goalFrames.push({ frame_id: frameId, sha256: sha256(raw), count: flow.count.goal });
  }
  flow.freshStatus = async () => flow.status();
  flow.waitFor = async (predicate, label) => {
    if (label.includes('frozen goal replay')) {
      flow.count.rgb += 2; flow.count.nativeRgb += 2;
      flow.latest.rgb.value.timestamp_ns = '2'; flow.latest.nativeRgb.value.timestamp_ns = '2';
      fixtureGoal(flow.status().goal_selection);
    }
    const value = predicate();
    assert(value, `in-memory exercise failed waiting for ${label}`);
    return value;
  };
  flow.ws = { send(frame) {
    assert.equal(frame[0], 1);
    assert.equal(frame.readUInt32LE(1), 1);
    const command = JSON.parse(decodeString(frame.subarray(5)));
    actions.push(command.action);
    const prior = flow.status();
    let next = { ...prior, epoch: prior.epoch + 1 };
    let result;
    if (command.action === 'capture_goal' || command.action === 'import_goal') {
      assert.equal(command.payload.expected_epoch, prior.epoch);
      const nextSelection = { ...selection, selection_id: `selection-${++selected}`, source: command.action === 'capture_goal' ? 'camera' : 'import' };
      next.goal_selection = nextSelection;
      fixtureGoal(nextSelection);
      result = { status: next, selection: nextSelection, goal: { rgb_jpeg_base64: raw.toString('base64') } };
    } else if (command.action === 'start') {
      assert.equal(command.payload.selection_id, prior.goal_selection.selection_id);
      assert(!Object.hasOwn(command.payload, 'goal'));
      next.state = 'WAITING_INPUT';
      next.task = { task_id: 'owned-fixture-task', backend: command.payload.backend, execution_mode: 'shadow',
        selection_id: command.payload.selection_id, goal_sha256: prior.goal_selection.sha256, goal_version: 1, timeout_s: 30 };
      next.preview = { identity: { task_id: next.task.task_id }, epoch: next.epoch, goal_version: 1,
        trajectory_body_m: [[0, 0], [0.1, 0]], backend: command.payload.backend, frame: 'fixture-local' };
      next.metrics = { inference_started: prior.metrics.inference_started + 1 };
      flow.latest.localPath = { value: { poses: 2, frame_id: 'imagegoal_model_local' } };
    } else if (command.action === 'pause') {
      next.state = 'PAUSED'; next.task = { ...prior.task, paused: true };
      flow.latest.localPath.value.poses = 0;
    } else if (command.action === 'resume') {
      next.state = 'WAITING_INPUT'; next.task = { ...prior.task, paused: false };
    } else if (command.action === 'cancel') {
      next.state = 'CANCELLED'; flow.latest.localPath.value.poses = 0;
    } else if (command.action === 'clear_goal') {
      next.goal_selection = { ...selection, mode: 'live', ready: false, selection_id: null, sha256: null };
    } else assert.fail(`unexpected fixture action ${command.action}`);
    flow.latest.status = { value: next };
    const pending = flow.pending.get(command.request_id);
    assert(pending, 'wire command has no acknowledgement waiter');
    clearTimeout(pending.timer); flow.pending.delete(command.request_id); pending.resolve(result ?? next);
  } };
  await flow.exerciseBackend('nomad');
  assert.deepEqual(actions, ['capture_goal', 'start', 'pause', 'resume', 'cancel', 'clear_goal', 'import_goal', 'clear_goal']);
  assert.equal(flow.report.exercises[0].stop_preserved_selection, true);
  assert.equal(flow.report.exercises[0].import.did_not_start, true);
  assert.equal(flow.report.exercises[0].final_live, true);
  assert.equal(flow.status().goal_selection.ready, false);
  console.log(JSON.stringify({ ok: true, self_test: 'CDR, native RGB subscription/decode, staged goal contracts, ownership guards, readonly guard, full in-memory capture/start/Stop/clear/import exercise', network_opened: false }));
}

export async function main(argv = process.argv.slice(2)) {
  const args = options(argv);
  if (!args) return 0;
  if (args.selfTest) { await selfTest(); return 0; }
  assert(typeof WebSocket === 'function', 'Node >=22 with built-in WebSocket is required');
  const report = { schema_version: 'imagegoal.foxglove_probe.v1', started_at: new Date().toISOString(), url: args.url, mode: args.exercise ? 'bounded_shadow_exercise' : 'read_only', ok: false, protocol_status: [], commands: [], exercises: [], checks: {} };
  const probe = new Probe(args.url, report);
  try {
    await probe.waitFor(() => probe.server, 'serverInfo');
    assert.equal(probe.ws.protocol, 'foxglove.sdk.v1');
    if (args.exercise) {
      assert(probe.server.capabilities?.includes('clientPublish'), 'bridge lacks clientPublish');
      assert(probe.server.supportedEncodings?.includes('cdr'), 'bridge lacks CDR client publishing');
    }
    await probe.waitFor(() => Object.keys(TOPICS).every((name) => [...probe.subscriptions.values()].some((channel) => channel.name === name)), 'all expected channels');
    await probe.waitFor(() => probe.statusVersion >= 2 && probe.count.rgb > 0 && probe.count.nativeRgb >= 2 && probe.count.localPath > 0 && probe.count.bridge > 0, 'fresh status, runtime/native RGB, bridge status, local path', 12000);
    const status = probe.status();
    assert(Date.now() - probe.latest.status.received_at_ms < 1500, 'runtime status subscription is stale');
    assert(Date.now() - probe.latest.rgb.received_at_ms < 2500, 'RGB subscription is stale');
    assert(Date.now() - probe.latest.nativeRgb.received_at_ms < 2500, 'native RGB subscription is stale');
    assert(status.sensors?.rgb && status.sensors.rgb.age_s >= 0 && status.sensors.rgb.age_s < 2, 'runtime RGB source is stale or absent');
    report.server = { name: probe.server.name, protocol: probe.ws.protocol, capabilities: probe.server.capabilities, supported_encodings: probe.server.supportedEncodings };
    report.checks = { channels: Object.fromEntries([...probe.subscriptions.values()].map((channel) => [channel.name, { topic: channel.topic, encoding: channel.encoding, schema: channel.schemaName }])), live_status: true, live_rgb_jpeg: true, live_native_rgb_jpeg: true, native_rgb_messages: probe.count.nativeRgb, local_path_messages: true, initial_goal_jpeg_received: (probe.count.goal || 0) > 0, goal_not_required_while_live: status.goal_selection?.mode === 'live', initial_selection: status.goal_selection, odom_available: status.sensors?.odom != null, synthetic_feedback: status.synthetic_feedback };
    if (args.exercise) {
      shadowGuard(status);
      assert(TERMINAL.has(status.state), `refusing to replace active ${status.state} task`);
      probe.exercise = true;
      probe.session = status.session_id;
      await probe.advertise();
      probe.advertised = true;
      await probe.exerciseBackend('nomad', args.snapshotDir);
      if (args.lightnav) await probe.exerciseBackend('lightnav', args.snapshotDir);
      // Clear ACK can precede the status subscription. Observe a fresh status
      // before describing the final canonical selection in the receipt.
      await probe.freshStatus();
    }
    report.ok = true;
  } catch (error) {
    report.error = error.message;
    if (probe.startAttempted) {
      try { await probe.stopOwned(); } catch (stopError) { report.cleanup = { ok: false, error: stopError.message }; }
    }
  } finally {
    report.finished_at = new Date().toISOString();
    report.message_counts = probe.count;
    // A live selection has no frozen goal. Historical goal publications can
    // remain cached after clear; the canonical status is the current truth.
    report.checks.goal_jpeg_received = (probe.count.goal || 0) > 0;
    const final = probe.status();
    if (final) report.final_runtime = { session_id: final.session_id, state: final.state, epoch: final.epoch, hardware_transport: final.hardware_transport, proposal_published_to_robot: final.proposal?.published_to_robot, synthetic_feedback: final.synthetic_feedback, rgb_age_s: final.sensors?.rgb?.age_s, odom_available: final.sensors?.odom != null, goal_selection: final.goal_selection, task: final.task && { task_id: final.task.task_id, backend: final.task.backend, execution_mode: final.task.execution_mode, goal_version: final.task.goal_version, selection_id: final.task.selection_id } };
    probe.close();
    if (args.report) await writeFile(args.report, JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify(report, null, 2));
  }
  return report.ok ? 0 : 1;
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  try { process.exitCode = await main(); } catch (error) { console.error(error.message); process.exitCode = 1; }
}
