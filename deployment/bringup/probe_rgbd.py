"""Check actual RGB-D messages without starting a camera or robot client."""
from __future__ import annotations
import argparse
import json
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-s',type=float,default=6)
    args=parser.parse_args()
    import rclpy
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[])
    node=rclpy.create_node('imagegoal_rgbd_preflight')
    latest={}
    subscriptions=[]
    def receive(kind,msg):
        latest[kind]={'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                      'frame':msg.header.frame_id,'width':msg.width,'height':msg.height,'encoding':msg.encoding}
    for kind,topic in [('rgb','/camera/camera/color/image_raw'),('depth','/camera/camera/aligned_depth_to_color/image_raw')]:
        subscriptions.append(node.create_subscription(Image,topic,lambda msg,k=kind:receive(k,msg),qos_profile_sensor_data))
    started=time.monotonic()
    ok=False
    try:
        while time.monotonic()-started<args.timeout_s:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(latest)<2: continue
            a,b=latest['rgb'],latest['depth'];now=time.time_ns()
            ok=(a['frame']==b['frame'] and a['width']==b['width'] and a['height']==b['height']
                and b['encoding']=='16UC1' and abs(a['stamp_ns']-b['stamp_ns'])<40_000_000
                and all(0<=(now-v['stamp_ns'])/1e9<.5 for v in latest.values()))
            if ok: break
        print(json.dumps({'ok':ok,'measured_messages':latest,'camera_started':False,'sdk_initialized':False}))
    finally:
        node.destroy_node();rclpy.shutdown()
    return 0 if ok else 1


if __name__=='__main__':
    raise SystemExit(main())
