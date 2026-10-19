# Candidate JetPack 6.2 iGPU base; validate on the actual BSP before use.
FROM nvcr.io/nvidia/pytorch:25.06-py3-igpu

WORKDIR /app
RUN python3 -m pip install --no-cache-dir \
    efficientnet-pytorch==0.7.1 diffusers==0.33.1 einops==0.8.1 \
    PyYAML==6.0.3 Pillow==11.2.1
COPY deployment/nomad /app/deployment/nomad

# Preserve NVIDIA's entrypoint. Source trees and weights are mounted read-only.
CMD ["python3", "-m", "deployment.nomad.server", \
     "--upstream", "/upstream/visualnav_transformer", \
     "--diffusion-policy", "/upstream/diffusion_policy", \
     "--checkpoint", "/models/nomad.pth", \
     "--checkpoint-sha256", "70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333", \
     "--device", "cuda", "--host", "0.0.0.0", "--port", "8091"]
