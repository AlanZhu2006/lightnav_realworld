# Sources and deployment adaptation

This directory is the NoMaD observation profile for the user's private Go2
deployment. The mature repositories below supplied the operational reference.
The NoMaD bridge configuration, compact layout, image relay and launchers were
written for this deployment. They do not include GEM's CEC workflow, operator
services or its static visualization transform.

## GEM-RealWorld

Repository: <https://github.com/AlanZhu2006/GEM-RealWorld>

Revision: `fd23b5d4006226e7a76b2c3f0bf03a8ec16e3436`

| Reference file | SHA-256 |
| --- | --- |
| `deployment/go2/config/foxglove_bridge.yaml` | `3e42da68f156617ac00875cf82a483bc7471b55988727325e99b157d781b039e` |
| `deployment/go2/config/navdp_debug.foxglove-layout.json` | `bfd09e5cdb1bef20d04e54125c34504b84cfb4eb030a6cd5429b0fb53d1e21ad` |
| `deployment/go2/scripts/run_foxglove_bridge.sh` | `43d15774ff836b452e2fa357d74e0232767d1109a92721a243a43731c9ceedd4` |
| `deployment/go2/foxglove_image_relay.py` | `c50ccabdb92a1048f24adcc8ae500985aabba00b6426a9b1bc043b3c9f3ec795` |

The checked-out revision has no root LICENSE/COPYING grant for its custom
deployment files. Its `THIRD_PARTY_NOTICES.md` describes separate research and
dependency terms. These source references do not assign a license to GEM or
its dependencies. The user has explicitly requested reuse of their mature
repositories for this private deployment. Re-evaluate distribution rights
before publishing copied GEM source or bundled dependencies.

## go2_tinynav

Repository: <https://github.com/AlanZhu2006/go2_tinynav>

Revision: `5705bb61dafb407594970ab2bc85c63fc71e0a24`

- Reference: `scripts/run_visible_vnc.sh`
- SHA-256: `96453d1f71216f85911c8963cdf8ce7c58dbbb087a07a0c10a34aa6d293f29ef`
- Included license: Apache-2.0, copyright 2025 UniflexAI.
- Original license SHA-256:
  `97a075922ae744d0b5d59c48bf9713ac72c4930b5879479f6d60eafa7d8daf48`.
- A complete license copy is retained as `LICENSE.TinyNav`.

`run_vnc_desktop.sh` preserves the independent virtual desktop approach and
uses parameterized project paths. It deliberately replaces process killing,
X-lock removal, generated passwords and LAN listening with occupied-display
rejection, an existing private password file and loopback listening. This
change notice is also present in the script.

## Interface references

- Foxglove ROS 2 bridge parameters:
  <https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md>
- Ubuntu 22.04 TigerVNC command-line interface:
  <https://manpages.ubuntu.com/manpages/jammy/man1/tigervncserver.1.html>

All ROS packages, Foxglove, TigerVNC, Xfce, OpenCV, SDKs and model artifacts
retain their own terms. None is bundled by this directory.
