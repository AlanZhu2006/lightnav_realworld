"""ROS RGB-D/odom ingress and shadow-only visualization egress.

This node never publishes TF, Odometry, Twist or SDK commands. Input RGB-D are
paired by their original headers; they retain exposure/system-mapped stamps.
HTTP runs in workers with bounded handoffs, not in ROS callbacks. Operator
commands delegate to the existing runtime and do not add transport authority.
"""
from __future__ import annotations

import argparse
import base64
from collections import deque
import io
import json
import math
from pathlib import Path as FilePath
import threading
import time
from urllib.request import urlopen
from urllib.parse import urlencode

import numpy as np
from PIL import Image

from deployment.nomad.shadow_client import request_json
from deployment.imagegoal.foxglove_control import OperatorControl, local_preview_poses
from deployment.imagegoal.preview import LatestFramePreview


def stamp_ns(header) -> int:
    return int(header.stamp.sec)*1_000_000_000+int(header.stamp.nanosec)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-url",default="http://127.0.0.1:8093")
    parser.add_argument("--rgb-topic",default="/camera/camera/color/image_raw")
    parser.add_argument("--preview-compressed-topic",help="optional native compressed camera stream used only for display")
    parser.add_argument("--preview-timestamp-source",help="provenance of that stream header; does not alter RGB-D ingress")
    parser.add_argument("--depth-topic",default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--odom-topic",default="/go2/odometry")
    parser.add_argument("--sample-hz",type=float,default=4.)
    parser.add_argument("--preview-hz",type=float,default=8.,help="independent live camera display rate; does not alter model RGB-D sampling")
    parser.add_argument("--preview-width",type=int,default=320)
    parser.add_argument("--preview-quality",type=int,default=60)
    parser.add_argument("--depth-scale-m",type=float,default=.001)
    parser.add_argument("--pair-tolerance-s",type=float,default=.04)
    parser.add_argument("--max-input-age-s",type=float,default=.3,help="drop already-old source pairs before encoding; never refresh their timestamps")
    parser.add_argument("--disable-operator-control",action="store_true",help="disable the bounded Foxglove command adapter")
    parser.add_argument("--duration-s",type=float,help="optional bounded observer validation interval")
    parser.add_argument("--report",type=FilePath,help="optional observer counters and actual source-age report on bounded shutdown")
    parser.add_argument("--timestamp-source",default="realsense_ros_header_system_time")
    parser.add_argument("--odom-timestamp-source",default="go2_dds_receive_host_system_time_not_sensor_source_time")
    args = parser.parse_args(argv)
    if min(args.sample_hz,args.preview_hz,args.preview_width,args.depth_scale_m,args.pair_tolerance_s,args.max_input_age_s)<=0 or not 1<=args.preview_quality<=95:
        parser.error("rates, width, depth scale and tolerances must be positive; preview quality must be 1..95")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,QoSHistoryPolicy,QoSReliabilityPolicy,QoSDurabilityPolicy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as ROSImage,CompressedImage
    from nav_msgs.msg import Odometry,Path
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    class Bridge(Node):
        def __init__(self):
            super().__init__("imagegoal_observer")
            self.cv = CvBridge()
            self.rgb_buffer,self.depth_buffer = deque(maxlen=8),deque(maxlen=8)
            self.condition = threading.Condition()
            self.pending_rgbd,self.pending_odom = None,None
            self.stop_event = threading.Event()
            self.next_sample_ns = 0
            self.last_rgb_stamp = 0
            self.counters = {key:0 for key in ("pairs","unpaired","sent_rgbd","sent_odom","ingress_errors","poll_errors","source_age_dropped","pending_pair_replaced")}
            self.status_pub = self.create_publisher(String,"/imagegoal/shadow/status",10)
            self.bridge_status_pub = self.create_publisher(String,"/imagegoal/shadow/bridge_status",10)
            self.operator_result_pub = self.create_publisher(String,"/imagegoal/operator/result",10)
            self.operator_control = None
            if not args.disable_operator_control:
                self.operator_control = OperatorControl(args.runtime_url,self.publish_operator_result,request_json=request_json)
                self.create_subscription(String,"/imagegoal/operator/command",self.operator_callback,10)
            self.local_path_pub = self.create_publisher(Path,"/imagegoal/shadow/local_path",10)
            display_qos=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,depth=1,reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                   durability=QoSDurabilityPolicy.VOLATILE)
            self.preview_pubs = {"rgb":self.create_publisher(CompressedImage,"/imagegoal/shadow/rgb/compressed",display_qos),
                                 "goal":self.create_publisher(CompressedImage,"/imagegoal/shadow/goal/compressed",2)}
            self.preview = LatestFramePreview(self.publish_live_rgb,hz=args.preview_hz,max_age_s=args.max_input_age_s)
            self.last_goal_preview = None
            self.goal_preview_at = 0.
            self.poll_index = 0
            self.sent_age_s=deque(maxlen=10000)
            self.path_pubs = {key:self.create_publisher(Path,"/imagegoal/shadow/"+topic,10) for key,topic in
                              (("trajectory_odom_m_rad","path"),("reference_odom_m_rad","reference"),("prediction_odom_m_rad","prediction"))}
            latest_qos=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,depth=1,reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                  durability=QoSDurabilityPolicy.VOLATILE)
            self.create_subscription(ROSImage,args.rgb_topic,self.rgb_callback,latest_qos)
            self.create_subscription(ROSImage,args.depth_topic,self.depth_callback,latest_qos)
            if args.preview_compressed_topic:
                self.create_subscription(CompressedImage,args.preview_compressed_topic,self.compressed_preview_callback,latest_qos)
            self.create_subscription(Odometry,args.odom_topic,self.odom_callback,latest_qos)
            self.worker = threading.Thread(target=self.send_loop,name="imagegoal-ros-http-ingress",daemon=True)
            self.worker.start()
            self.poll_worker = threading.Thread(target=self.poll_loop,name="imagegoal-ros-observation-poll",daemon=True)
            self.poll_worker.start()
            self.get_logger().info("ImageGoal RGB-D/odom observer with bounded runtime operator controls; local preview is unanchored; no robot transport")

        def publish_operator_result(self,value):
            message = String()
            message.data = json.dumps(value,allow_nan=False,separators=(",",":"))
            self.operator_result_pub.publish(message)

        def operator_callback(self,message):
            if self.operator_control is not None:
                self.operator_control.submit(message.data)

        def rgb_callback(self,message):
            # Display is independent of RGB-D pairing/inference. Its bounded
            # latest-only worker never queues old video behind model requests.
            if not args.preview_compressed_topic:
                self.preview.submit(message,stamp_ns(message.header))
            stamp = stamp_ns(message.header)
            if stamp<=self.last_rgb_stamp or stamp<self.next_sample_ns:
                return
            self.rgb_buffer.append(message)
            self.try_pair()

        def compressed_preview_callback(self,message):
            # The native camera already feeds runtime RGB. This is display-only,
            # with its original RPC-receive header and no duplicate ingress.
            self.preview.submit(message,stamp_ns(message.header))

        def depth_callback(self,message):
            self.depth_buffer.append(message)
            self.try_pair()

        def try_pair(self):
            if not self.rgb_buffer or not self.depth_buffer:
                return
            candidates=[]
            for rgb_message in self.rgb_buffer:
                rgb_stamp=stamp_ns(rgb_message.header)
                if rgb_stamp<=self.last_rgb_stamp or rgb_stamp<self.next_sample_ns:
                    continue
                depth_message=min(self.depth_buffer,key=lambda image:abs(stamp_ns(image.header)-rgb_stamp))
                delta=abs(stamp_ns(depth_message.header)-rgb_stamp)/1e9
                if delta<=args.pair_tolerance_s:
                    candidates.append((rgb_stamp,rgb_message,depth_message,delta))
            if not candidates:
                self.counters["unpaired"] += 1
                return
            # A delayed depth callback can match an earlier buffered RGB.
            # Always choosing only latest RGB can phase-lock the two streams
            # one camera period apart and starve otherwise valid pairs.
            stamp,rgb,depth,delta=max(candidates,key=lambda item:item[0])
            if rgb.header.frame_id!=depth.header.frame_id:
                self.get_logger().error("aligned RGB/depth optical frame mismatch")
                return
            ages=[(time.time_ns()-stamp_ns(message.header))/1e9 for message in (rgb,depth)]
            if any(age<-.05 or age>args.max_input_age_s for age in ages):
                self.counters["source_age_dropped"] += 1
                self.rgb_buffer.clear()
                return
            with self.condition:
                if self.pending_rgbd is not None:
                    self.counters["pending_pair_replaced"] += 1
                self.pending_rgbd=(rgb,depth,delta)
                self.condition.notify()
            self.counters["pairs"] += 1
            self.last_rgb_stamp=stamp
            period=int(1e9/args.sample_hz)
            self.next_sample_ns=self.next_sample_ns+period if self.next_sample_ns else stamp+period
            if self.next_sample_ns<=stamp:
                self.next_sample_ns=stamp+period
            self.rgb_buffer.clear()

        def encode_pair(self,pair):
            rgb,depth,delta=pair
            ages=[(time.time_ns()-stamp_ns(message.header))/1e9 for message in (rgb,depth)]
            if any(age<-.05 or age>args.max_input_age_s for age in ages):
                self.counters["source_age_dropped"] += 1
                return None
            try:
                rgb_array = self.cv.imgmsg_to_cv2(rgb,desired_encoding="rgb8")
                depth_array = self.cv.imgmsg_to_cv2(depth,desired_encoding="passthrough")
                if depth.encoding!="16UC1" or depth_array.dtype!=np.uint16 or depth_array.ndim!=2 or depth_array.shape!=rgb_array.shape[:2]:
                    raise ValueError("expected aligned 16UC1 depth with matching RGB dimensions")
                jpeg,png = io.BytesIO(),io.BytesIO()
                Image.fromarray(rgb_array).save(jpeg,format="JPEG",quality=95,subsampling=0)
                Image.fromarray(depth_array).save(png,format="PNG",compress_level=1)
                if max((time.time_ns()-stamp_ns(message.header))/1e9 for message in (rgb,depth))>args.max_input_age_s:
                    self.counters["source_age_dropped"] += 1
                    return None
                common = {"clock_domain":"unix_epoch_ns","timestamp_source":args.timestamp_source,"synthetic":False,"frame_name":rgb.header.frame_id}
                return ({**common,"frame_id":f"ros-rgb-{stamp_ns(rgb.header)}","capture_timestamp_ns":stamp_ns(rgb.header),"rgb_jpeg_base64":base64.b64encode(jpeg.getvalue()).decode(),
                         "provenance":{"source":"ROS Image original header","rgb_topic":args.rgb_topic,"depth_topic":args.depth_topic,"pair_delta_s":delta}},
                        {**common,"capture_timestamp_ns":stamp_ns(depth.header),"depth_png_base64":base64.b64encode(png.getvalue()).decode(),
                         "scale_m":args.depth_scale_m,"aligned_to_rgb":True})
            except Exception as exc:
                self.counters["ingress_errors"] += 1
                self.get_logger().error(f"RGB-D encoding rejected: {exc}")
                return None

        def odom_callback(self,message):
            q = message.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            position = message.pose.pose.position
            value = {"capture_timestamp_ns":stamp_ns(message.header),"clock_domain":"unix_epoch_ns",
                     "timestamp_source":args.odom_timestamp_source,"frame_name":message.header.frame_id,"synthetic":False,
                     "body_frame_name":message.child_frame_id,"commissioning":False,
                     "pose_m_rad":[position.x,position.y,yaw],"velocity_v_w":[message.twist.twist.linear.x,message.twist.twist.angular.z]}
            with self.condition:
                self.pending_odom = value
                self.condition.notify()

        def send_loop(self):
            while not self.stop_event.is_set():
                with self.condition:
                    if self.pending_odom is None and self.pending_rgbd is None:
                        self.condition.wait(.2)
                    odom,pair = self.pending_odom,self.pending_rgbd
                    self.pending_odom,self.pending_rgbd = None,None
                try:
                    if odom:
                        request_json(args.runtime_url+"/inputs/odom",timeout_s=.5,payload=odom)
                        self.counters["sent_odom"] += 1
                    if pair:
                        pair=self.encode_pair(pair)
                    if pair:
                        # Both retain their original source stamps. Runtime
                        # gates can briefly hold during this atomic pair handoff.
                        request_json(args.runtime_url+"/inputs/depth",timeout_s=.5,payload=pair[1])
                        request_json(args.runtime_url+"/inputs/rgb",timeout_s=.5,payload=pair[0])
                        self.counters["sent_rgbd"] += 1
                        self.sent_age_s.append((time.time_ns()-pair[0]["capture_timestamp_ns"])/1e9)
                except Exception as exc:
                    self.counters["ingress_errors"] += 1
                    self.get_logger().warning(f"runtime ingress failed; newest data will replace old: {exc}")

        def poll_loop(self):
            while not self.stop_event.is_set():
                started=time.monotonic()
                self.poll()
                self.stop_event.wait(max(0,.4-(time.monotonic()-started)))

        def poll(self):
            self.poll_index += 1
            try:
                status,_ = request_json(args.runtime_url+"/status",timeout_s=.25)
                text = String()
                text.data = json.dumps(status,allow_nan=False,separators=(",",":"))
                self.status_pub.publish(text)
                plan = status.get("plan")
                for key,publisher in self.path_pubs.items():
                    path = Path()
                    path.header.frame_id = plan.get("odom_frame_id","imagegoal_no_odom") if plan else "imagegoal_no_odom"
                    path.header.stamp = self.get_clock().now().to_msg()
                    for x,y,yaw in (plan.get(key,[]) if plan else []):
                        pose = PoseStamped()
                        pose.header = path.header
                        pose.pose.position.x,pose.pose.position.y = float(x),float(y)
                        pose.pose.orientation.z,pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
                        path.poses.append(pose)
                    publisher.publish(path)
                # This body-local path is intentionally separate from odom
                # paths and has no TF that could imply world anchoring. Clear
                # stale previews after task/goal/epoch changes or Stop.
                local_path = Path()
                local_path.header.frame_id = "imagegoal_model_local"
                local_path.header.stamp = self.get_clock().now().to_msg()
                for x,y,yaw in local_preview_poses(status):
                    pose = PoseStamped()
                    pose.header = local_path.header
                    pose.pose.position.x,pose.pose.position.y = x,y
                    pose.pose.orientation.z,pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
                    local_path.poses.append(pose)
                self.local_path_pub.publish(local_path)
                if self.poll_index%2==0:
                    self.publish_goal_preview(status)
            except Exception:
                self.counters["poll_errors"] += 1
            summary = String()
            summary.data = json.dumps({"not_for_motion":True,"hardware_transport":"absent","counters":self.counters,
                                       "rgb_topic":args.rgb_topic,"depth_topic":args.depth_topic,"odom_topic":args.odom_topic,
                                       "preview":{**self.preview.snapshot(),"max_hz":args.preview_hz,"source_topic":args.preview_compressed_topic or args.rgb_topic,
                                                  "timestamp_source":args.preview_timestamp_source or args.timestamp_source,
                                                  "source_exposure_timestamp_available":not bool(args.preview_compressed_topic),
                                                  "original_header_preserved":True}})
            self.bridge_status_pub.publish(summary)

        def publish_live_rgb(self,source):
            if isinstance(source,CompressedImage):
                from deployment.imagegoal.preview import decode_compressed_rgb
                image = decode_compressed_rgb(source.data)
            else:
                rgb = self.cv.imgmsg_to_cv2(source,desired_encoding="rgb8")
                image = Image.fromarray(rgb)
            if image.width>args.preview_width:
                image.thumbnail((args.preview_width,max(1,round(image.height*args.preview_width/image.width))),Image.Resampling.BILINEAR)
            buffer = io.BytesIO()
            image.save(buffer,format="JPEG",quality=args.preview_quality)
            if (time.time_ns()-stamp_ns(source.header))/1e9>args.max_input_age_s:
                return False
            message = CompressedImage()
            message.header = source.header
            message.format = "jpeg"
            message.data = buffer.getvalue()
            self.preview_pubs["rgb"].publish(message)
            return True

        def publish_goal_preview(self,status):
            # Frozen goals are low-rate state, not live video. Republish every
            # five seconds for new subscribers, or immediately on goal change.
            selection = status.get("goal_selection") or {}
            selection_id = selection.get("selection_id")
            if not selection.get("ready") or not isinstance(selection_id,str):
                self.last_goal_preview = None
                return
            identity = json.dumps([status.get("session_id"),selection_id],separators=(",",":"))
            if identity==self.last_goal_preview and time.monotonic()-self.goal_preview_at<5.:
                return
            try:
                with urlopen(args.runtime_url+"/goal.jpg?"+urlencode({"selection_id":selection_id}),timeout=.25) as response:
                    if response.headers.get("X-ImageGoal-Selection-Id") != selection_id:
                        raise ValueError("goal preview selection identity mismatch")
                    content = response.read(4*1024*1024+1)
                if len(content)>4*1024*1024:
                    raise ValueError("goal preview JPEG is too large")
                message = CompressedImage()
                # This frozen goal's stamp is deliberately display time, not
                # a claim about the current camera source exposure.
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = "imagegoal_frozen_goal_"+selection_id
                message.format = "jpeg"
                message.data = content
                self.preview_pubs["goal"].publish(message)
                self.last_goal_preview,self.goal_preview_at = identity,time.monotonic()
            except Exception:
                pass

        def shutdown(self):
            self.stop_event.set()
            self.preview.close()
            with self.condition:
                self.condition.notify_all()
            if self.operator_control is not None:
                self.operator_control.close()
            self.worker.join(timeout=2)
            self.poll_worker.join(timeout=2)
            if args.report:
                import statistics
                value={"not_for_motion":True,"hardware_transport":"absent","counters":self.counters,
                       "source_age_at_handoff_s":{"count":len(self.sent_age_s),"median":statistics.median(self.sent_age_s) if self.sent_age_s else None,
                                                  "max":max(self.sent_age_s) if self.sent_age_s else None},
                       "qos":"BEST_EFFORT KEEP_LAST depth1","pair_tolerance_s":args.pair_tolerance_s,
                       "max_input_age_s":args.max_input_age_s,"original_headers_preserved":True}
                args.report.parent.mkdir(parents=True,exist_ok=True)
                args.report.write_text(json.dumps(value,indent=2)+"\n")

    rclpy.init()
    node = Bridge()
    deadline=time.monotonic()+args.duration_s if args.duration_s else None
    try:
        while rclpy.ok() and (deadline is None or time.monotonic()<deadline):
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__=="__main__":
    raise SystemExit(main())
