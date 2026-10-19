# 0.1.11

Redesign the native Foxglove panel as an English Navigation Console with a quieter visual hierarchy, compact task cards, grouped telemetry, and separate mission/runtime information. Follow Foxglove dark/light themes and adapt to panel width. Translate controls, empty states, validation, acknowledgements and trajectory labels; present common runtime gate codes in plain English and preserve original diagnostics. Keep the registered `ImageGoal` panel ID, ROS topics, command payloads, capability gates, goal identity checks, native supervision limits, priority Stop, and bounded image decoding unchanged. No new model capability or robot task is enabled by this update.

# 0.1.10

Add an explicit **LightNav ImageGoal** Go2 adaptation beside NoMaD ImageGoal, official ObjNav/VLN navigation and tracking. Require a canonical frozen goal and an explicit runtime adapter capability; extra image-goal text is optional and stored separately. Keep official language/tracking payloads free of goal pixels and selection IDs. Preserve 0.1.9 demand-based goal subscriptions and camera decode behavior. The HTTP operator captures/imports a canonical goal before starting adaptation. Add camera/import, native/shadow, missing-goal/capability and mode-isolation tests.

# 0.1.9

Subscribe to the original frozen-goal JPEG only while ImageGoal needs to restore it. Unsubscribe after a successful decode and during language navigation or tracking, retaining the live RGB/status/result subscriptions. Restore the goal subscription when its session or selection changes. This removes repeated full-image transfers from a constrained video connection without changing goal pixels, selection identity, model inputs, or robot controls.

# 0.1.8

Replace experimental LightNav ImageGoal with official RGB-history-and-language navigation. Expose explicit ObjNav/VLN modes, require a trimmed instruction, hide image-goal capture controls for LightNav, and remove goal pixels/selection IDs from its native and shadow payloads. Reject legacy LightNav ImageGoal requests. Preserve NoMaD image goals, tracking, fixed native budgets, priority Stop and session/epoch guards. Add no-image navigation DOM/payload checks for Foxglove and the HTTP operator page.

# 0.1.7

Add **LightNav 跟随** beside the two ImageGoal tasks. Start native or shadow tracking directly from RGB and a required target description without freezing or sending a goal image. Keep tracking and ImageGoal text separately, require explicit runtime task-mode capabilities, show target visibility and safe hold, and preserve one-run supervision, active-executor/pending-operation guards and priority native Stop. Mode selection and restored settings never start a task. Add native/shadow no-goal, capability, stale/read-only, visibility/hold and late-start Stop regression checks.

# 0.1.6

Allow **启动 LightNav 真机** through the same native task/Go2 executor used by NoMaD. Send the selected backend and optional instruction with the frozen selection; retain the 60-second supervised budget and independent priority stop. Use runtime-supported backend capabilities, preserve older NoMaD-only runtime compatibility, and label native execution using the actual task backend. Preserve shadow-only ordinary Start, full goal capture/import, decoded-image gating, and bandwidth-limited preview. Add browser checks for both native backends, LightNav start/stop races and old-runtime compatibility.

# 0.1.5

Add an explicit **真机启动 · NoMaD** button for the current frozen full goal with a fixed 60-second supervised budget, alongside the existing shadow controls. Require runtime supervision capabilities and keep LightNav in shadow mode. Add independent **立即停止真机** with native stop priority and late-start acknowledgement invalidation. Show executor process state separately from command authorization. Keep the single ROS connection and add a deterministic offline packaging fallback for Jetsons without Node, with JavaScript parsing, model regressions and panel interaction checks.

## 0.1.4

Fix camera flicker and SSH bandwidth regression while preserving the canonical frozen-goal workflow introduced in 0.1.3. Subscribe only to the bandwidth-limited shadow RGB stream; preserve header/status source provenance. Decode new frames offscreen, atomically replace the display, retain last good frames on failures, bound pending decodes, and share committed frames with the live goal without repeated source resets.

## 0.1.3

- Live target preview, acknowledged camera capture / original JPEG import, and canonical frozen selections before Start.
- Recapture atomically stops the old task and restores live preview; ordinary Stop retains the selected target.
- Reconnection accepts only the image matching the runtime selection ID; Stop, session changes, and camera changes invalidate stale operation acknowledgements.
- Native Go2 RGB preview follows the runtime camera source and labels host RPC receive timestamps explicitly.

# 0.1.0

Native ImageGoal dashboard for NoMaD/LightNav shadow tasks, including acknowledged controls, raw JPEG goal capture, current-identity local preview, frame-safe odom/MPC rendering, and visible source freshness.
