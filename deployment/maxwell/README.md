# Optional shared GPU server

Source `environment.sh` to keep all environments, weights, caches, logs and
installers under `/local_data/$(id -un)`; `HOME` remains unchanged. The preparation
script hides GPUs and does not authorize inference. `launch.py --gpu GPU_ID`
requires an explicitly coordinated single GPU and refuses a busy GPU unless the
operator has separately authorized sharing. The workstation remains the live
endpoint until an operator validates and switches a stopped task installation.

Set `MAXWELL_SSH_DESTINATION` privately before using `jetson_link.sh`, and provision
a dedicated restricted forwarding key and pinned host key under `.runtime/ssh`.
No account address, private key or trusted host identity is distributed here.
The tunnel uses only loopback ports: Jetson standby model 18885, server operator
HTTP 18093 and Foxglove 18766. It neither switches the model URL nor starts motion.

See [portable deployment](../../docs/REALWORLD_GO2_CONSOLE.md). This directory
contains templates, not a snapshot of an already prepared server.
