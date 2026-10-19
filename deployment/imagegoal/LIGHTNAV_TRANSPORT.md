# LightNav observation transport

The transport runs the released Go2 client's original `_encode_image` from
LightNav-0 commit `3015508b70fb6e30bbd66701e94c9b916dcb8df8`: OpenCV decodes the
compressed source JPEG, resizes with `INTER_AREA` to **480 wide × 270 high**,
and encodes JPEG at **quality 80**. The complete upstream file and license are
vendored under `vendor/lightnav_official_client`; a SHA256 check precedes AST
loading of the original encoder, frame class and constants. The model retains
its original checkpoint preprocessing after this transport step.

These are the official client's observation derivatives, with their codec,
OpenCV version, source hash and wire hash recorded. Original camera JPEGs are
preserved separately. The previous custom 448×256 bilinear/JPEG-90 transport
has been removed.

Use the helper only for the runtime's LightNav observation branch:

```python
# Initialize once per runtime.
self.lightnav_transport = LightNavObservationTransport(cache_frames=64)

# After ingesting a new frame, outside the runtime/task lock, only while the
# runtime loop and an unpaused, non-stopped LightNav task are active:
self.lightnav_transport.prefetch(frame)

# In _dispatch, after selecting backend and frames:
observations = [frame.wire() for frame in frames]
transport_audit = None
if self.task["backend"] == "lightnav":
    observations, transport_audit = self.lightnav_transport.observations(frames)
payload["observations"] = observations
metadata["observation_transport"] = transport_audit

# On runtime shutdown:
self.lightnav_transport.close()
```

Official language navigation/tracking sends observations and an instruction,
without a goal JPEG. The separate Go2 ImageGoal profile sends the complete
original goal JPEG alongside those observations; this encoder never modifies
that goal. Keep the original JPEG recordings.
`Frame.wire()` remains unchanged for NoMaD. Do not change capture timestamps,
frame IDs, source metadata, or provenance. Record the returned audit beside the
request so each wire SHA and source SHA can be distinguished. The server's
existing parser accepts the extra `rgb_transport` record without a model or
protocol change.

The bounded content-addressed cache avoids repeatedly decoding overlapping
frames. A recomputed frame after cache eviction produces identical bytes on
the same encoder stack. Do not change codec settings or runtime encoder
versions midway through an existing LightNav policy session; begin a new task.
Policy history validates identical overlap pixels. Pointing coordinates from
the server now reference the 480×270 wire image; use `wire_size_wh` and
`source_size_wh` to scale overlays onto the full source image.

Prefetch is compression only: one lazy daemon thread and a queue of at most 16
JPEG references. `prefetch(frame)` uses nonblocking lock/queue operations and
returns false when closed, busy, duplicate, or full; ordinary `observations()`
still encodes a missed frame. Separate cache, encoding, and producer locks let
ingestion continue during encoding and prevent foreground/background duplicate
encoding. `prefetch_status()` and each transport audit expose queue bounds,
completions, failures, and the latest bounded error record. A failed JPEG does
not terminate the worker. `close(timeout_s=1.)` rejects later enqueueing,
discards queued work, and waits at most the specified timeout for any active
encode to finish. The worker performs no HTTP, inference, or robot control.

CPU-only checks:

```bash
.venv-nomad-server/bin/python -m unittest deployment.imagegoal.audit_lightnav_transport -v
.venv-nomad-server/bin/python -m unittest deployment.imagegoal.audit_lightnav_dispatch -v
```

The tests cover malformed/truncated/non-JPEG input, source preservation,
NoMaD original wire bytes, source identities, cache eviction/replay, prefetch
concurrency, and byte equality with the complete official client encoder.
No model, GPU inference, robot SDK, network service, or robot task is started.

Earlier transport benchmarks under `lightnav_transport_20261019` describe the
previous codec and do not characterize this official encoder. Current source
hashes and CPU conformance results are recorded under
`data/realworld/lightnav_official_alignment_20261019/`.
