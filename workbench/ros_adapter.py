"""ROS IO on its own executor thread. Never opens hardware on construction."""
import json,threading,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rcl_interfaces.msg import Log,Parameter,ParameterValue
from rcl_interfaces.srv import SetParametersAtomically,GetParameters
from cv_bridge import CvBridge

class RosAdapter(Node):
    def __init__(self,service):
        super().__init__('piper_workbench_service');self.s=service;self.bridge=CvBridge();self.pubs={};self.joints=None;self.tcp=None;self.seen={};self.rates={};self.nodes=[]
        self.geometry={};self.tcp_geometry={};self.closed=False
        from tf2_ros import Buffer,TransformListener
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.subs=[]
        for cam,raw,det in [('d455','/camera/camera/color/image_raw','/detection_info'),('dabai','/dabai/color/image_raw','/dabai/detection_info')]:
            self.subs.extend([self.create_subscription(Image,raw,lambda m,c=cam:self.image(c,m),qos_profile_sensor_data),
                self.create_subscription(String,det,lambda m,c=cam:self.detection(c,m),10)])
        self.subs.append(self.create_subscription(String,'/workbench/vision_geometry',self.geometry_cb,10))
        self.subs.extend([self.create_subscription(JointState,'/feedback/joint_states',self.feedback,qos_profile_sensor_data),
            self.create_subscription(PoseStamped,'/feedback/tcp_pose',self.pose,qos_profile_sensor_data),
            self.create_subscription(String,'/sorting/cycle_result',self.result,10),
            self.create_subscription(String,'/sorting_status',lambda m:self.s.emit('task.status',{'action_state':m.data}),10),
            self.create_subscription(Log,'/rosout',self.rosout,100)])
        self._executor=rclpy.executors.MultiThreadedExecutor(num_threads=2);self._executor.add_node(self)
        self.thread=threading.Thread(target=self._executor.spin,daemon=True);self.thread.start()
        threading.Thread(target=self.probe_tcp,daemon=True).start()
    def geometry_cb(self,msg):
        try:
            data=json.loads(msg.data)
            for cam,row in data.items():
                if cam not in ('d455','dabai'):continue
                parent='base_link' if cam=='d455' else 'link6'
                child='camera_color_optical_frame' if cam=='d455' else 'dabai_color_optical_frame'
                try:
                    t=self.tf_buffer.lookup_transform(parent,child,rclpy.time.Time()).transform
                    row.update(tf_position=[t.translation.x,t.translation.y,t.translation.z],tf_orientation=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
                except Exception:pass
                self.geometry[cam]=dict(row,stamp=time.time())
        except (ValueError,TypeError):pass
    def probe_tcp(self):
        import xml.etree.ElementTree as ET
        while rclpy.ok() and not self.closed:
            try:
                manifest=json.loads((self.s.root/'config/calibration/manifest.json').read_text())
                if any(x['kind']=='tcp' for x in manifest.get('workbench_selections',{}).values()):
                    row={}
                    from handeye_calibration_ros.tcp_verification_ros import startup_offset
                    row['sdk_start']=startup_offset(self.s.root)['offset']
                    for name,param,key in [('/agx_arm_ctrl_single_node','tcp_offset','sdk_parameter'),('/move_group','robot_description','moveit'),('/piper_workbench_io','robot_description','workbench')]:
                        value=self.call(GetParameters,name+'/get_parameters',GetParameters.Request(names=[param])).values[0]
                        if param=='tcp_offset':row[key]=list(value.double_array_value)
                        else:
                            model=ET.fromstring(value.string_value)
                            joint=next(j for j in model.findall('joint') if j.find('child') is not None and j.find('child').get('link')=='tcp_link')
                            if joint.get('type')!='fixed' or joint.find('parent').get('link')!='link6':raise ValueError('TCP 模型父坐标系不一致')
                            origin=joint.find('origin');row[key]=[float(v) for v in (origin.get('xyz','0 0 0')+' '+origin.get('rpy','0 0 0')).split()]
                    self.tcp_geometry=dict(row,stamp=time.time())
            except Exception:self.tcp_geometry={}
            # Small interruptible wait; no ROS executor is blocked by parameter probes.
            for _ in range(20):
                if not rclpy.ok() or self.closed:return
                time.sleep(.1)
    def calibration_evidence(self):return dict(self.geometry,tcp=self.tcp_geometry)
    def image(self,camera,msg):
        try:
            now=time.time();last=self.seen.get(camera,now);self.seen[camera]=now
            if now>last:self.rates[camera]=.9*self.rates.get(camera,0)+.1/(now-last)
            self.s.capture.update(camera,self.bridge.imgmsg_to_cv2(msg,'bgr8'),msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        except Exception as exc:self.s.logs.add(camera,str(exc))
    def detection(self,camera,msg):
        try:
            data=json.loads(msg.data)
            if data.get('camera',camera)!=camera:return
            if camera=='d455' and self.s.direction_samples is not None:
                for obj in data.get('objects',[]):
                    bp=obj.get('base_position_m')
                    if bp:
                        key=(data.get('motion_session'),obj.get('track_id'))
                        self.s.direction_samples.setdefault(key,[]).append((float(data.get('header_stamp',time.time())),float(bp['x']),float(bp['y'])))
            now=time.time();previous=self.seen.get('yolo:'+camera,now)
            if now>previous:self.rates['yolo:'+camera]=.9*self.rates.get('yolo:'+camera,0)+.1/(now-previous)
            self.s.tasks.detection(camera,data);self.seen['yolo:'+camera]=now
            self.s.emit('vision.targets',{'camera_id':camera,'detection':data})
        except Exception as exc:self.s.logs.add('检测消息',str(exc))
    def feedback(self,msg):
        values=dict(zip(msg.name,msg.position));keys=['joint'+str(i) for i in range(1,7)]
        if all(k in values for k in keys):self.joints=([values[k] for k in keys],time.time());self.seen['arm']=time.time()
    def pose(self,msg):
        p=msg.pose.position;q=msg.pose.orientation
        self.tcp=([p.x,p.y,p.z,q.x,q.y,q.z,q.w],time.time())
    def capture_actual_pose(self):
        if not self.joints or time.time()-self.joints[1]>.5:raise ValueError('等待新鲜的实际关节反馈 /feedback/joint_states')
        return dict(joints_rad=self.joints[0],tcp_xyzw=self.tcp[0] if self.tcp and time.time()-self.tcp[1]<.5 else None,stamp=self.joints[1])
    def result(self,msg):
        try:self.s.tasks.result(json.loads(msg.data))
        except Exception as exc:self.s.logs.add('任务结果',str(exc))
    def rosout(self,msg):
        stamp=msg.stamp.sec+msg.stamp.nanosec*1e-9
        self.s.log_ros(msg.name,msg.msg,stamp)
    def send(self,topic,data):
        if topic not in self.pubs:self.pubs[topic]=self.create_publisher(String,topic,10)
        self.pubs[topic].publish(String(data=json.dumps(data,ensure_ascii=False) if not isinstance(data,str) else data))
    def call(self,typ,name,request):
        client=self.create_client(typ,name)
        try:
            if not client.wait_for_service(timeout_sec=2.):raise RuntimeError('服务未运行: '+name)
            future=client.call_async(request);event=threading.Event();future.add_done_callback(lambda f:event.set())
            if not event.wait(5.):raise RuntimeError('服务响应超时: '+name)
            return future.result()
        finally:self.destroy_client(client)
    def parameters(self,node,values):
        params=[]
        for name,value in values.items():
            if isinstance(value,bool):v=ParameterValue(type=1,bool_value=value)
            elif isinstance(value,int):v=ParameterValue(type=2,integer_value=value)
            elif isinstance(value,float):v=ParameterValue(type=3,double_value=value)
            else:raise ValueError('仅支持数值与布尔参数: '+name)
            params.append(Parameter(name=name,value=v))
        result=self.call(SetParametersAtomically,node+'/set_parameters_atomically',SetParametersAtomically.Request(parameters=params)).result
        if not result.successful:raise RuntimeError(result.reason)
        return {'state':'applied_at_next_frame','values':values}
    def camera_exposure(self,camera,values):
        if camera=='d455':
            params={'rgb_camera.enable_auto_exposure':bool(values['auto'])}
            if not values['auto']:params['rgb_camera.exposure']=int(values['value'])
            return self.parameters('/camera/camera',params)
        # Original Orbbec exposure service types and names are declared in CAMERA_DEFAULTS.
        from orbbec_camera_msgs.srv import SetInt32
        from std_srvs.srv import SetBool
        auto=bool(values['auto'])
        response=self.call(SetBool,'/dabai/set_color_auto_exposure',SetBool.Request(data=auto))
        if not response.success:raise RuntimeError(response.message)
        if not auto:
            response=self.call(SetInt32,'/dabai/set_color_exposure',SetInt32.Request(data=int(values['value'])))
            if not response.success:raise RuntimeError(response.message)
        return {'state':'applied','values':values}
