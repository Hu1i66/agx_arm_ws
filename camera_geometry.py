"""Image-time camera transforms, backed by the existing ROS tf2 tree."""
from collections import deque
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from runtime_geometry import transform, inverse, tcp_transform
import runtime_config as rc


class CameraGeometry:
    def __init__(self,node):
        import rclpy.time
        import tf2_ros
        from geometry_msgs.msg import PoseStamped
        self.node=node;self.S=tcp_transform();self.calibrations={};self.poses=deque(maxlen=2400);self.lock=threading.Lock()
        self.buffer=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buffer,node)
        self.subscription=node.create_subscription(PoseStamped,'/feedback/tcp_pose',self._pose,10)
        from rcl_interfaces.srv import GetParameters
        self._GetParameters=GetParameters
        self._driver_client=node.create_client(GetParameters,'/agx_arm_ctrl_single_node/get_parameters')
        self._driver_future=None
        self._driver_values=None
        self._driver_checked_at=0.
        self._poll_timer=node.create_timer(.5,self._poll_tcp)

    def _poll_tcp(self):
        if self._driver_future is not None and self._driver_future.done() and not self._driver_future.exception():
            self._driver_values=self._driver_future.result().values
            self._driver_checked_at=time.monotonic()
        if self._driver_future is None or self._driver_future.done():
            if self._driver_client.service_is_ready():
                request=self._GetParameters.Request();request.names=['tcp_offset']
                self._driver_future=self._driver_client.call_async(request)


    def _pose(self,msg):
        if msg.header.frame_id not in ('','base_link'):return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        p=msg.pose.position;q=msg.pose.orientation
        try:T=transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        except ValueError:return
        with self.lock:self.poses.append((stamp,T,time.monotonic()))

    def camera(self,camera,stamp):
        import rclpy.time
        if camera not in self.calibrations:
            he=rc.load_handeye(camera)
            self.calibrations[camera]=(he,transform(he['position'],he['orientation']))
        he,C=self.calibrations[camera]
        if he['parent']=='base_link':return C.copy()
        if not stamp or not np.isfinite(stamp):raise ValueError('Image has no valid timestamp')
        try:
            msg=self.buffer.lookup_transform('base_link',he['parent'],rclpy.time.Time(seconds=float(stamp)))
            p=msg.transform.translation;q=msg.transform.rotation
            return transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])@C
        except Exception:
            # SDK pose timestamps provide an independent history when TF trails the image.
            values=self._driver_values
            if values is None or time.monotonic()-self._driver_checked_at>1.5:raise ValueError('Fresh driver TCP configuration unavailable')
            if len(values)!=1 or values[0].type!=8:raise ValueError('Driver TCP unavailable')
            offset=np.asarray(values[0].double_array_value,dtype=float)
            if offset.shape!=(6,) or not np.allclose(transform(offset[:3],Rotation.from_euler('xyz',offset[3:]).as_quat()),self.S,atol=1e-8,rtol=0):
                raise ValueError('T1 must be restarted with active TCP before Dabai feedback conversion')
            with self.lock:rows=list(self.poses)
            if not rows or time.monotonic()-rows[-1][2]>1.:raise ValueError('Fresh image-time robot feedback unavailable')
            before=[row for row in rows if row[0]<=stamp];after=[row for row in rows if row[0]>=stamp]
            if not before or not after:raise ValueError('Image-time robot pose not bracketed; retry after fresh feedback')
            a,b=before[-1],after[0]
            if max(stamp-a[0],b[0]-stamp)>.05:raise ValueError('Robot/image timestamps differ by more than 50ms')
            u=0. if b[0]==a[0] else (stamp-a[0])/(b[0]-a[0])
            T=np.eye(4);T[:3,3]=(1-u)*a[1][:3,3]+u*b[1][:3,3]
            T[:3,:3]=a[1][:3,:3] if b[0]==a[0] else Slerp([a[0],b[0]],Rotation.from_matrix([a[1][:3,:3],b[1][:3,:3]]))([stamp]).as_matrix()[0]
            if he['parent']=='link6':T=T@inverse(self.S)
            return T@C


def image_qos(node, topic, camera):
    """Use offered reliability without creating incompatible RGB/depth subscriptions."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    offers = node.get_publishers_info_by_topic(topic)
    best_effort = (any(p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for p in offers)
                   if offers else camera == 'dabai')
    return QoSProfile(depth=2, reliability=(ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE))


def fresh_image(node, stamp):
    if stamp is None or not np.isfinite(stamp) or stamp <= 0:
        return False
    age = node.get_clock().now().nanoseconds * 1e-9 - stamp
    return -.05 <= age <= 1.
