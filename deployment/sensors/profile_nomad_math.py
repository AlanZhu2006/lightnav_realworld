"""Bounded saved-RGB comparison of explicit CUDA math configurations.

Does not change the running model service or the robot. Numerical differences
are reported against the same seed/input in strict FP32, not navigation scores.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import statistics
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--goal',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repeats',type=int,default=3)
    args=parser.parse_args()
    if not 1<=args.repeats<=10: parser.error('repeats must be 1..10')
    args.output.mkdir(parents=True,exist_ok=False)
    import numpy as np
    import torch
    from PIL import Image
    from deployment.nomad.policy import NoMaDPolicy,RGBFrame,InferenceIdentity
    project=Path(__file__).resolve().parents[2]
    policy=NoMaDPolicy(upstream=project/'third_party/visualnav_transformer',
        diffusion_policy=project/'third_party/diffusion_policy',checkpoint=project/'assets/nomad/nomad.pth',
        device='cuda',expected_checkpoint_sha256='70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333')
    manifest=json.loads(args.manifest.read_text())
    observations=[RGBFrame(Image.open(args.manifest.parent/f['path']).convert('RGB'),f['frame_id'],f['capture_timestamp_ns']) for f in manifest['frames'][:4]]
    goal=Image.open(args.goal).convert('RGB')
    identity=InferenceIdentity('math-profile','saved-rgb','same-goal',0,observations[-1].frame_id)
    rows=[];reference=None
    for name,benchmark,tf32 in [('strict_fp32',False,False),('autotuned_fp32',True,False),('autotuned_tf32',True,True)]:
        torch.backends.cudnn.benchmark=benchmark
        torch.backends.cudnn.allow_tf32=tf32
        torch.backends.cuda.matmul.allow_tf32=tf32
        samples=[];times=[];cold_ms=None
        for iteration in range(args.repeats+1):
            torch.manual_seed(260806);torch.cuda.synchronize();start=time.perf_counter()
            result=policy.predict(observations,goal,identity)
            torch.cuda.synchronize();elapsed=(time.perf_counter()-start)*1000
            if iteration==0: cold_ms=elapsed
            else: times.append(elapsed);samples.append(result['trajectory_body_m'])
        value=np.asarray(samples[-1])
        if reference is None: reference=value.copy()
        row={'mode':name,'warm_ms':times,'warm_median_ms':statistics.median(times),'initial_ms':cold_ms,
             'max_abs_path_difference_m':float(np.max(np.abs(value-reference))),
             'finite':bool(np.isfinite(value).all()),'trajectory_body_m':value.tolist(),
             'cudnn_benchmark':benchmark,'allow_tf32':tf32}
        rows.append(row);print(json.dumps(row),flush=True)
    report={'checkpoint':policy.contract['checkpoint'],'device':torch.cuda.get_device_name(),
            'modes':rows,'running_service_changed':False,'camera_opened':False,'robot_sdk_initialized':False,
            'navigation_quality_verified':False,'same_seed_each_call':True}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__': main()
