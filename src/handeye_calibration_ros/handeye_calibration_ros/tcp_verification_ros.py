"""ROS adapter for explicit TCP verification. No arm lifecycle or scene mutation APIs."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

try:
    from . import tcp_verification as verify
    from . import tcp_calibration as cal
except ImportError:
    import tcp_verification as verify
    import tcp_calibration as cal


def startup_offset(workspace):
    """Read the managed driver's launch parameter file, not its mutable parameter server."""
    import sys
    if str(workspace) not in sys.path: sys.path.insert(0,str(workspace))
    from runtime_process_manager import ProcessManager, _pgid_members
    manager=ProcessManager(workspace)
    record=manager.get_record('arm')
    if not record or not manager.verify_identity(record)[0]:
        raise ValueError('T1须通过统一arm入口启动，才能核对SDK启动时的TCP')
    found=[]
    class LaunchLoader(yaml.SafeLoader): pass
    LaunchLoader.add_constructor('tag:yaml.org,2002:python/tuple',lambda loader,node: loader.construct_sequence(node))
    for pid in _pgid_members(record['pgid']):
        try:
            argv=Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
            if not any(Path(arg).name=='agx_arm_ctrl_single' for arg in argv[:2]): continue
            values={}; paths=[]
            for i,arg in enumerate(argv[:-1]):
                if arg=='--params-file':
                    path=Path(argv[i+1]); contents=yaml.load(path.read_text(),Loader=LaunchLoader)
                    for name,block in contents.items():
                        if name in ('/**','agx_arm_ctrl_single_node','/agx_arm_ctrl_single_node'):
                            values.update(block.get('ros__parameters',{}))
                    paths.append(str(path))
            offset=cal.finite_array(values['tcp_offset'],(6,),'驱动启动TCP').tolist()
            found.append(dict(pid=pid,offset=offset,parameter_files=paths))
        except (OSError,KeyError,ValueError,yaml.YAMLError): continue
    if len(found)!=1: raise ValueError('无法唯一读取实际驱动的启动TCP参数，拒绝仅凭在线参数确认')
    return found[0]


class RosBackend:
    def __init__(self,assessment,options,stop,report):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_action_status_default
        from geometry_msgs.msg import PoseStamped, Pose
        from sensor_msgs.msg import JointState
        from agx_arm_msgs.msg import AgxArmStatus
        from action_msgs.msg import GoalStatusArray
        from moveit_msgs.srv import GetCartesianPath,GetPositionFK,GetStateValidity,GetPlanningScene,GetPositionIK
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import RobotState,RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterEvent
        self.rclpy=rclpy;self.Pose=Pose;self.RobotState=RobotState;self.RobotTrajectory=RobotTrajectory
        self.Point=JointTrajectoryPoint;self.GetParameters=GetParameters
        self.Cart=GetCartesianPath;self.IK=GetPositionIK;self.FK=GetPositionFK;self.Valid=GetStateValidity;self.Scene=GetPlanningScene
        self.Execute=ExecuteTrajectory;self.ActionClient=ActionClient
        self.assessment,self.options,self.stop,self.report=assessment,options,stop,report
        self.workspace=Path(__file__).resolve().parents[3]
        self.lock=threading.Lock();self.latest={};self.parameter_changed=False;self.active_goal=None
        self.pending_goal=None;self.cancel_future=None;self.result_future=None;self.cancel_requested=False
        self.shutdown_event=threading.Event();self.thread=None;self.node=None;self.executor=None
        rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        try:
            self.node=Node('piper_tcp_verification')
            self.node.create_subscription(PoseStamped,'/feedback/tcp_pose',lambda m:self.receive('pose',m),10)
            self.node.create_subscription(JointState,'/feedback/joint_states',lambda m:self.receive('joints',m),10)
            self.node.create_subscription(JointState,'/control/joint_states',lambda m:self.receive('control',m),10)
            self.node.create_subscription(AgxArmStatus,'/feedback/arm_status',lambda m:self.receive('arm',m),10)
            self.node.create_subscription(GoalStatusArray,'/arm_controller/follow_joint_trajectory/_action/status',
                                          lambda m:self.receive('controller_status',m),qos_profile_action_status_default)
            self.node.create_subscription(GoalStatusArray,'/execute_trajectory/_action/status',
                                          lambda m:self.receive('execute_status',m),qos_profile_action_status_default)
            self.node.create_subscription(ParameterEvent,'/parameter_events',self.parameter_event,10)
            self.cart=self.node.create_client(GetCartesianPath,'/compute_cartesian_path')
            self.ik=self.node.create_client(GetPositionIK,'/compute_ik')
            self.fk=self.node.create_client(GetPositionFK,'/compute_fk')
            self.valid=self.node.create_client(GetStateValidity,'/check_state_validity')
            self.scene=self.node.create_client(GetPlanningScene,'/get_planning_scene')
            self.execute_client=None
            self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
            def spin():
                try:
                    while not self.shutdown_event.is_set(): self.executor.spin_once(timeout_sec=.05)
                except Exception as exc:
                    self.report['receiver_error']=str(exc);self.stop.set()
            self.thread=threading.Thread(target=spin,daemon=True);self.thread.start()
        except BaseException:
            self.close();raise

    def receive(self,key,message):
        now=time.monotonic()
        stamp=None
        if hasattr(message,'header'):
            stamp=message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        with self.lock:
            prior=self.latest.get(key)
            # Repeated queued/source-frozen stamps do not refresh physical feedback age.
            fresh=prior[2] if stamp is not None and prior and stamp==prior[3] else now
            self.latest[key]=(message,now,fresh,stamp)

    def parameter_event(self,message):
        if message.node=='/agx_arm_ctrl_single_node' and any(p.name=='tcp_offset' for p in
                list(message.changed_parameters)+list(message.new_parameters)+list(message.deleted_parameters)):
            # Ignore matching declarations before preflight; a change after it invalidates SDK assumptions.
            if hasattr(self,'limits'): self.parameter_changed=True

    def wait(self,future,timeout=10.,monitor=False):
        deadline=time.monotonic()+timeout
        while not future.done():
            if self.stop.is_set(): raise InterruptedError('用户中断')
            if time.monotonic()>deadline: raise TimeoutError('ROS请求超时')
            if monitor: self.fresh_feedback()
            time.sleep(.002)
        if future.exception() is not None: raise ValueError('ROS请求失败: '+str(future.exception()))
        return future.result()

    def call(self,client,request,timeout=10.):
        if not client.wait_for_service(timeout_sec=2.): raise ValueError('服务未就绪: '+client.srv_name)
        if self.stop.is_set(): raise InterruptedError('请求前中断')
        return self.wait(client.call_async(request),timeout)

    def params(self,node,names):
        client=self.node.create_client(self.GetParameters,node+'/get_parameters')
        try:
            request=self.GetParameters.Request();request.names=names
            response=self.call(client,request)
            if response is None or len(response.values)!=len(names): raise ValueError('参数响应不完整: '+node)
            return response.values
        finally: self.node.destroy_client(client)

    def fresh_feedback(self):
        now=time.monotonic()
        with self.lock: values=dict(self.latest)
        for key in ('pose','joints','control','arm'):
            if key not in values or now-values[key][2]>1.: raise ValueError('真实反馈/控制状态过期: '+key)
        pose=values['pose'][0]
        if pose.header.frame_id not in ('','base_link'): raise ValueError('TCP反馈不是base_link')
        p,q=pose.pose.position,pose.pose.orientation
        actual=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        cal.tcp_to_flange(actual[:3],actual[3:],[0.]*6)  # Finite/nonzero quaternion validation.
        joint=values['joints'][0];control=values['control'][0]
        mapping={n:float(v) for n,v in zip(joint.name,joint.position)}
        commands={n:float(v) for n,v in zip(control.name,control.position)}
        for data in (mapping,commands):
            cal.finite_array([data[n] for n in verify.JOINTS],(6,),'关节反馈')
        arm=values['arm'][0]
        if arm.ctrl_mode!=1 or arm.arm_status!=0 or arm.err_status!=0:
            raise ValueError(f'机械臂不是正常CAN控制状态: mode={arm.ctrl_mode} status={arm.arm_status} err={arm.err_status}')
        for name in ('joint_angle_limit','communication_status_joint'):
            flags=list(getattr(arm,name,[]))
            if len(flags)!=6 or any(flags): raise ValueError('机械臂故障/反馈不完整: '+name)
        return dict(pose=actual,joints=mapping,control=commands,received_monotonic=values['pose'][1],
                    source_stamp_ns=values['pose'][3],joint_source_stamp_ns=values['joints'][3],legacy_empty_frame=not bool(pose.header.frame_id))

    def state(self,joints):
        state=self.RobotState();state.joint_state.name=list(joints)
        state.joint_state.position=[float(joints[n]) for n in joints];state.is_diff=True  # Preserve the planning scene's attached collision bodies.
        return state

    def pose_message(self,pose):
        message=self.Pose()
        message.position.x,message.position.y,message.position.z=map(float,pose[:3])
        message.orientation.x,message.orientation.y,message.orientation.z,message.orientation.w=map(float,pose[3:])
        return message

    def preflight(self):
        deadline=time.monotonic()+5.
        while True:
            try: start=self.fresh_feedback();break
            except ValueError:
                if time.monotonic()>deadline: raise
                if self.stop.is_set(): raise InterruptedError('启动被中断')
                time.sleep(.05)
        nodes=set(self.node.get_node_names())
        if nodes.intersection({'sorting_headless_client_node','piper_moveit_action_client'}):
            raise ValueError('请结束动作后关闭T5和T6，禁止与自动/动态/排队任务并用')
        arm_workspace=Path(getattr(self.options,'arm_workspace',None) or self.workspace).expanduser().resolve()
        startup=startup_offset(arm_workspace)
        startup['workspace']=str(arm_workspace)
        values=self.params('/agx_arm_ctrl_single_node',['tcp_offset','arm_type'])
        if values[0].type!=8 or values[1].type!=4 or not values[1].string_value.startswith('piper'):
            raise ValueError('仅支持Piper，且tcp_offset必须为double_array')
        online=cal.finite_array(list(values[0].double_array_value),(6,),'在线TCP')
        expected=np.asarray(self.assessment['tcp_offset'])
        if not np.allclose(online,expected,atol=1e-8,rtol=0) or not np.allclose(startup['offset'],expected,atol=1e-8,rtol=0):
            raise ValueError('T1启动TCP/在线TCP与候选不一致；须由操作者用验证参数重新启动T1，不能在线param set')
        description,semantic=self.params('/move_group',['robot_description','robot_description_semantic'])
        if description.type!=4 or semantic.type!=4: raise ValueError('MoveIt模型参数不可读取')
        root=ET.fromstring(description.string_value)
        joint=next(
            (j for j in root.findall('joint') if j.find('child') is not None and j.find('child').get('link')=='tcp_link'),None)
        if joint is None or joint.get('type')!='fixed' or joint.find('parent').get('link')!='link6':
            raise ValueError('MoveIt模型tcp_link不是link6的固定工具端点')
        origin=joint.find('origin')
        xyz=list(map(float,origin.get('xyz','0 0 0').split()));rpy=list(map(float,origin.get('rpy','0 0 0').split()))
        if not np.allclose(cal.offset_transform(xyz+rpy),cal.offset_transform(expected),atol=1e-8,rtol=0):
            raise ValueError('MoveIt模型工具变换与T1启动/候选不一致')
        semantic_root=ET.fromstring(semantic.string_value)
        self.allowed_pairs={frozenset((c.get('link1'),c.get('link2'))) for c in semantic_root.findall('disable_collisions')}
        self.limits={}
        names=[]
        for n in verify.JOINTS:
            for field in ('has_velocity_limits','max_velocity','has_acceleration_limits','max_acceleration'):
                names.append('robot_description_planning.joint_limits.'+n+'.'+field)
        planned=self.params('/move_group',names)
        for index,n in enumerate(verify.JOINTS):
            model=root.find(f"./joint[@name='{n}']")
            limit=model.find('limit') if model is not None else None
            vflag,vmax,aflag,amax=planned[index*4:index*4+4]
            if limit is None or not vflag.bool_value or not aflag.bool_value or vmax.type!=3 or amax.type!=3:
                raise ValueError('缺少真实URDF/MoveIt关节速度加速度限制: '+n)
            self.limits[n]=dict(lower=float(limit.get('lower')),upper=float(limit.get('upper')),
                                velocity=min(float(limit.get('velocity')),vmax.double_value),acceleration=amax.double_value)
        if getattr(self.options,'sweep',False):
            interpolation=self.params('/arm_controller',['interpolation_method'])[0]
            if interpolation.type!=4 or interpolation.string_value!='splines':
                raise ValueError('连续摆动要求arm_controller interpolation_method=splines')
        self.report['preflight']=dict(driver_startup=startup,t1_tcp=online.tolist(),
            moveit_tcp_origin=xyz+rpy,limits=self.limits,collision_checking=True,planning_link='tcp_link',group='arm')
        self.check_scene()
        self.guard()
        fk=self.compute_fk(start['joints'])
        error=verify.pose_error(start['pose'],fk)
        if error['position_mm']>self.options.position_tolerance_mm or error['orientation_deg']>2:
            raise ValueError('SDK真实TCP反馈与MoveIt FK模型不一致: '+str(error))
        # Require stationary real state at initialization, not only stationary command values.
        time.sleep(.3);next_state=self.fresh_feedback()
        if verify.pose_error(start['pose'],next_state['pose'])['position_mm']>1 or np.max(np.abs(
                np.array([start['joints'][n] for n in verify.JOINTS])-np.array([next_state['joints'][n] for n in verify.JOINTS])))>.01:
            raise ValueError('起点仍在运动')
        return next_state

    def check_scene(self):
        request=self.Scene.Request();request.components.components=502  # robot state, attached objects, world, octomap, TF, ACM, padding/scaling
        scene=self.call(self.scene,request).scene
        matrix=scene.allowed_collision_matrix
        if any(matrix.default_entry_values): raise ValueError('场景存在默认放行碰撞，拒绝执行验证')
        for i,row in enumerate(matrix.entry_values):
            for j,enabled in enumerate(row.enabled):
                if enabled and i!=j and frozenset((matrix.entry_names[i],matrix.entry_names[j])) not in self.allowed_pairs:
                    raise ValueError('场景ACM包含非SRDF碰撞放行；请恢复正常场景后重试')
        from rclpy.serialization import serialize_message
        parts=[serialize_message(scene.world),serialize_message(matrix)]
        # Unspecified joints are filled from MoveIt's scene by IK/state-validity services.
        # Include them even when the hardware feedback omits a gripper joint.
        nonarm=sorted((name,float(value)) for name,value in zip(
            scene.robot_state.joint_state.name,scene.robot_state.joint_state.position) if name not in verify.JOINTS)
        parts.append(json.dumps(nonarm,allow_nan=False).encode())
        multi=copy.deepcopy(scene.robot_state.multi_dof_joint_state)
        multi.header.stamp.sec=0;multi.header.stamp.nanosec=0
        parts.append(serialize_message(multi))
        for obj in scene.robot_state.attached_collision_objects: parts.append(serialize_message(obj))
        for original in scene.fixed_frame_transforms:
            transform=copy.deepcopy(original);transform.header.stamp.sec=0;transform.header.stamp.nanosec=0
            parts.append(serialize_message(transform))
        parts.append(str([(p.link_name,p.padding) for p in scene.link_padding]).encode())
        parts.append(str([(p.link_name,p.scale) for p in scene.link_scale]).encode())
        self.scene_hash=hashlib.sha256(b''.join(len(p).to_bytes(8,'big')+p for p in parts)).hexdigest()

    def trajectory_proof(self,plan):
        return hashlib.sha256(json.dumps(dict(points=plan['points'],target=plan['target'],
            joints=plan['joint_names'],nonarm_state=plan.get('validated_nonarm_state',{})),sort_keys=True,allow_nan=False).encode()).hexdigest()


    def wait_for_stationary(self):
        """Require a 0.3s window of distinct, fresh hardware joint feedback before bridging."""
        deadline=time.monotonic()+3.;history=[];last_stamp=None
        while True:
            if self.stop.is_set(): raise InterruptedError('等待关节停稳时中断')
            if self.parameter_changed: raise ValueError('等待停稳时TCP参数改变')
            current=self.fresh_feedback();now=time.monotonic()
            stamp=current['joint_source_stamp_ns']
            if stamp is not None and stamp!=last_stamp:
                history.append((now,np.array([current['joints'][n] for n in verify.JOINTS])))
                last_stamp=stamp
            while len(history)>2 and history[1][0]<=now-.3: history.pop(0)
            if len(history)>=3 and history[-1][0]-history[0][0]>=.3:
                span=np.ptp(np.array([row[1] for row in history]),axis=0)
                if float(np.max(span))<=1e-4:
                    self.report.setdefault('start_stability_checks',[]).append(dict(
                        joint_range_rad=span.tolist(),window_s=history[-1][0]-history[0][0],
                        distinct_joint_feedback=len(history),timestamp=datetime_iso()))
                    return current
            if now>=deadline: raise ValueError('3秒内关节未停稳或未收到足够新的关节反馈，未派发运动')
            time.sleep(.02)

    def guard(self,plan=None):
        if self.stop.is_set(): raise InterruptedError('验证已中断')
        verify.verify_input_hashes(self.assessment)
        if self.parameter_changed: raise ValueError('运行中TCP参数改变，SDK/模型一致性不可再保证')
        if {n for n in self.node.get_node_names()}.intersection({'sorting_headless_client_node','piper_moveit_action_client'}):
            raise ValueError('检测到T5/T6重新启动')
        current=self.fresh_feedback()
        if max(abs(current['joints'][n]-current['control'][n]) for n in verify.JOINTS)>.03:
            raise ValueError('ros2_control控制状态与真实关节反馈不同步，拒绝跳跃起步')
        idle_deadline=time.monotonic()+1.
        while True:
            with self.lock: statuses={k:self.latest.get(k) for k in ('controller_status','execute_status')}
            busy=[key for key,value in statuses.items() if value and any(s.status in (1,2,3) for s in value[0].status_list)]
            if not busy: break
            if time.monotonic()>idle_deadline: raise ValueError('控制器/MoveIt仍有活动轨迹: '+str(busy))
            if self.stop.is_set(): raise InterruptedError('等待控制器空闲时中断')
            self.fresh_feedback();time.sleep(.03)
        if plan is not None:
            current=self.wait_for_stationary()
            if max(abs(current['joints'][n]-plan['start_state'][n]) for n in verify.JOINTS)>.025:
                raise ValueError('真实起点已偏离规划起点')
            if getattr(self.options,'continuous_round',False):
                self.check_scene()
                same_start=max(abs(current['joints'][n]-plan['points'][0]['positions'][i]) for i,n in enumerate(verify.JOINTS))<=1e-8
                same_nonarm={k:float(v) for k,v in current['joints'].items() if k not in verify.JOINTS}==plan.get('validated_nonarm_state',{})
                if (same_start and same_nonarm and plan.get('checked_scene_hash')==self.scene_hash
                        and plan.get('trajectory_proof')==self.trajectory_proof(plan)):
                    plan['dispatch_start']=dict(current['joints']);plan['reused_unchanged_path_proof']=True
                    return current
            # Account for actual->planned first point with a timed, checked bridge.
            if 'planned_positions' not in plan:
                plan['planned_positions']=[list(p['positions']) for p in plan['points']]
            positions=[[current['joints'][n] for n in verify.JOINTS]] + plan['planned_positions']
            plan['points']=self.retime_positions(positions)
            if plan['points'][-1]['time_s']>60.: raise ValueError('含起点衔接的轨迹超过60秒')
            plan['dispatch_start']=dict(current['joints'])
            self.check_scene()
            before=getattr(self,'scene_hash',None)
            self.validate_path(plan,current['joints'])
            if getattr(self.options,'continuous_round',False):
                self.check_scene()
                if before!=self.scene_hash: raise ValueError('复核期间场景改变，须重新规划')
                plan['checked_scene_hash']=self.scene_hash;plan['trajectory_proof']=self.trajectory_proof(plan)
        return current

    def compute_fk(self,joints):
        request=self.FK.Request();request.header.frame_id='base_link';request.fk_link_names=['tcp_link']
        request.robot_state=self.state(joints)
        result=self.call(self.fk,request)
        if result.error_code.val!=1 or result.fk_link_names!=['tcp_link'] or len(result.pose_stamped)!=1:
            raise ValueError('FK失败')
        p=result.pose_stamped[0].pose
        return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]

    def validate_path(self,plan,base_state):
        maximum=0.;samples=0;orientation_extrema={}
        for position in verify.path_samples(plan['points']):
            state=dict(base_state);state.update(zip(verify.JOINTS,map(float,position)))
            request=self.Valid.Request();request.robot_state=self.state(state);request.group_name='arm'
            if not self.call(self.valid,request).valid: raise ValueError('规划/插值状态碰撞或无效')
            fk=self.compute_fk(state)
            error=verify.pose_error(plan['target']['pose'],fk)
            maximum=max(maximum,error['position_mm']);samples+=1
            if error['position_mm']>1.:
                self.report['path_failure']=dict(error=error,fk=fk,reference=plan['target']['pose'],joints=state,interpolation=plan.get('interpolation'))
                raise ValueError('规划TCP位置漂移超过1mm: '+str(error['position_mm']))
            if 'amplitude_deg' in plan['target']:
                excursion=verify.pose_error(plan['target']['reference_pose'],fk)['orientation_deg']
                if excursion>plan['target']['amplitude_deg']+.05: raise ValueError('整轮插值超出指定姿态幅度')
                reference=verify.Rotation.from_quat(plan['target']['reference_pose'][3:])
                vector=np.degrees((reference.inv()*verify.Rotation.from_quat(fk[3:])).as_rotvec())
                for axis in plan['target']['axis'].split('/'):
                    value=float(vector['xyz'.index(axis)]);entry=orientation_extrema.setdefault(axis,dict(min_deg=0.,max_deg=0.))
                    entry['min_deg']=min(entry['min_deg'],value);entry['max_deg']=max(entry['max_deg'],value)
        endpoint=dict(base_state);endpoint.update(zip(verify.JOINTS,plan['points'][-1]['positions']))
        if verify.pose_error(plan['target']['pose'],self.compute_fk(endpoint))['orientation_deg']>.5:
            raise ValueError('规划末端姿态未达到目标')
        if 'amplitude_deg' in plan['target']:
            amplitude=plan['target']['amplitude_deg']
            for axis,entry in orientation_extrema.items():
                if entry['min_deg']>-amplitude+.05 or entry['max_deg']<amplitude-.05:
                    raise ValueError('整轮路径未覆盖指定正负幅度: '+axis)
        plan['validated_nonarm_state']={k:float(v) for k,v in base_state.items() if k not in verify.JOINTS}
        plan['path_checks']=dict(collision_checked=True,interpolation_states=samples,max_model_position_error_mm=maximum,
                                  model_orientation_extrema_deg=orientation_extrema)

    def retime_positions(self,positions):
        if getattr(self.options,'continuous_round',False):
            try:
                return verify.retime_round(positions,self.limits,self.options.speed,self.options.motion_seconds,self.workspace)
            except ValueError:
                self.report['timing_failure']=dict(positions=positions,limits=self.limits,speed=self.options.speed,requested_seconds=self.options.motion_seconds)
                raise
        return verify.retime(positions,verify.JOINTS,self.limits,self.options.speed,smooth=getattr(self.options,'sweep',False))

    def ik_round_positions(self,seed,target):
        # Humble Cartesian service applies its own coarse TOTG blend before returning positions.
        # Use collision-aware MoveIt IK samples so our small-tolerance retiming starts from exact poses.
        previous=target['reference_pose'];state=dict(seed);positions=[[seed[n] for n in verify.JOINTS]]
        solved=0
        for endpoint in target['waypoints']:
            r0=verify.Rotation.from_quat(previous[3:]);r1=verify.Rotation.from_quat(endpoint[3:])
            delta=(r0.inv()*r1).as_rotvec()
            count=max(1,int(math.ceil(math.degrees(float(np.linalg.norm(delta)))/.5)))
            for part in range(1,count+1):
                self.fresh_feedback()
                fraction=part/count
                pose=(np.asarray(previous[:3])*(1-fraction)+np.asarray(endpoint[:3])*fraction).tolist()+(
                    r0*verify.Rotation.from_rotvec(delta*fraction)).as_quat().tolist()
                request=self.IK.Request();query=request.ik_request
                query.group_name='arm';query.ik_link_name='tcp_link';query.avoid_collisions=True
                query.robot_state=self.state(state);query.pose_stamped.header.frame_id='base_link'
                query.pose_stamped.pose=self.pose_message(pose);query.timeout.sec=0;query.timeout.nanosec=500000000
                response=self.call(self.ik,request,timeout=3.)
                if response.error_code.val!=1: raise ValueError('整轮IK/碰撞失败，拒绝不完整路径: '+str(response.error_code.val))
                mapping=dict(zip(response.solution.joint_state.name,response.solution.joint_state.position))
                q=cal.finite_array([mapping[n] for n in verify.JOINTS],(6,),'整轮IK结果')
                if np.max(np.abs(q-np.asarray(positions[-1])))>.12: raise ValueError('整轮IK关节跳跃超过0.12rad')
                for i,n in enumerate(verify.JOINTS):
                    if not self.limits[n]['lower']<=q[i]<=self.limits[n]['upper']: raise ValueError('IK关节越限: '+n)
                state.update(zip(verify.JOINTS,q.tolist()));positions.append(q.tolist());solved+=1
                if solved%60==0: print(f'只规划：已求解{solved}个带碰撞检查姿态，尚未发送运动',flush=True)
            previous=endpoint
        self.report['round_ik']=dict(complete=True,solved_samples=solved,angular_step_deg=.5,avoid_collisions=True)
        return positions

    def plan(self,seed,target):
        self.fresh_feedback()
        if getattr(self.options,'continuous_round',False):
            positions=self.ik_round_positions(seed,target);fraction=1.
        else:
            request=self.Cart.Request();request.header.frame_id='base_link';request.group_name='arm';request.link_name='tcp_link'
            request.start_state=self.state(seed);request.waypoints=[self.pose_message(p) for p in target.get('waypoints',[target['pose']])]
            request.max_step=.002;request.jump_threshold=5.;request.revolute_jump_threshold=.12
            request.prismatic_jump_threshold=.005;request.avoid_collisions=True
            response=self.call(self.cart,request,timeout=20.)
            if response.error_code.val!=1 or not math.isfinite(response.fraction) or response.fraction<1.-1e-9:
                raise ValueError('笛卡尔路径不完整或IK/碰撞失败: fraction='+str(response.fraction))
            trajectory=response.solution.joint_trajectory
            if list(trajectory.joint_names)!=verify.JOINTS: raise ValueError('规划返回非Piper六轴轨迹')
            positions=[list(p.positions) for p in trajectory.points]
            fraction=float(response.fraction)
            if not positions or np.max(np.abs(np.asarray(positions[0])-np.array([seed[n] for n in verify.JOINTS])))>1e-6:
                raise ValueError('规划路径起点不一致')
        points=self.retime_positions(positions)
        if points[-1]['time_s']>60.: raise ValueError('单步规划耗时过长，拒绝执行')
        plan=dict(interpolation='cubic' if getattr(self.options,'sweep',False) else 'quintic_zero_knots',target=target,start_state=dict(seed),end_state=dict(seed),fraction=fraction,
                  joint_names=verify.JOINTS,points=points)
        if getattr(self.options,'continuous_round',False): plan['planned_positions']=positions
        plan['end_state'].update(zip(verify.JOINTS,points[-1]['positions']))
        self.report['planning_in_progress']=plan
        if getattr(self.options,'continuous_round',False):
            self.check_scene();before=self.scene_hash
        self.validate_path(plan,seed)
        if getattr(self.options,'continuous_round',False):
            self.check_scene()
            if before!=self.scene_hash: raise ValueError('规划核查期间场景改变，须重新规划')
            plan['checked_scene_hash']=self.scene_hash;plan['trajectory_proof']=self.trajectory_proof(plan)
        self.report.pop('planning_in_progress',None)
        label=f"±{target['amplitude_deg']:g}deg整轮" if 'amplitude_deg' in target else f"{target['angle_deg']:+.1f}deg"
        print(f"已规划 {target['step']}: {target['axis']} {label}, {len(points)}点, {points[-1]['time_s']:.2f}s")
        return plan

    def execute(self,plan,stop):
        from action_msgs.msg import GoalStatus
        if self.execute_client is None:
            self.execute_client=self.ActionClient(self.node,self.Execute,'/execute_trajectory')
        if not self.execute_client.wait_for_server(timeout_sec=3.): raise ValueError('ExecuteTrajectory服务不可用')
        if stop.is_set(): raise InterruptedError('发送前中断')
        verify.verify_input_hashes(self.assessment)
        for attempt in range(4):
            if stop.is_set(): raise InterruptedError('衔接复查时中断')
            if self.parameter_changed: raise ValueError('goal派发前TCP参数改变')
            verify.verify_input_hashes(self.assessment)
            actual=self.fresh_feedback()
            delta=[actual['joints'][n]-plan['dispatch_start'][n] for n in verify.JOINTS]
            maximum=max(map(abs,delta))
            if maximum<=1e-4: break
            self.report.setdefault('start_revalidation',[]).append(dict(step=plan['target']['step'],
                attempt=attempt+1,delta_rad=delta,max_delta_rad=maximum,
                checked_start=dict(plan['dispatch_start']),actual_joints=dict(actual['joints']),timestamp=datetime_iso()))
            if attempt==3:
                raise ValueError(f'衔接复查3次后起点仍在变化: max_delta={maximum:.8f}rad，未派发后续运动')
            print(f'起点更新{maximum:.8f}rad，等待停稳并重新核查衔接（{attempt+1}/3），尚未派发本段',flush=True)
            self.guard(plan)
        trajectory=self.RobotTrajectory();trajectory.joint_trajectory.joint_names=verify.JOINTS
        for point in plan['points']:
            msg=self.Point();msg.positions=point['positions'];msg.velocities=point['velocities'];msg.accelerations=point['accelerations']
            ns=round(point['time_s']*1e9);msg.time_from_start.sec=ns//1000000000;msg.time_from_start.nanosec=ns%1000000000
            trajectory.joint_trajectory.points.append(msg)
        goal=self.Execute.Goal();goal.trajectory=trajectory
        self.cancel_requested=False;self.cancel_future=None;self.active_goal=None;self.result_future=None
        feedback_target=plan['target'].get('feedback_reference_pose',plan['target']['pose'])
        samples=[]
        self.report['inflight_step']=dict(step=plan['target']['step'],status='awaiting_acceptance',
            target_pose=feedback_target,model_target_pose=plan['target']['pose'],samples=samples,started_at=datetime_iso(),
            dispatched_monotonic=time.monotonic())
        if stop.is_set(): raise InterruptedError('goal派发前中断')
        if self.parameter_changed: raise ValueError('goal派发前TCP参数改变')
        dispatch_time=time.monotonic()
        self.pending_goal=self.execute_client.send_goal_async(goal)
        def accepted(future):
            try:
                handle=future.result()
                if handle is not None and handle.accepted:
                    self.active_goal=handle
                    self.result_future=handle.get_result_async()
                    if self.cancel_requested: self.cancel_future=handle.cancel_goal_async()
            except Exception as exc: self.report['goal_acceptance_error']=str(exc)
        self.pending_goal.add_done_callback(accepted)
        handle=self.wait(self.pending_goal,timeout=5.,monitor=True)
        if handle is None or not handle.accepted: raise ValueError('执行goal被拒绝')
        self.active_goal=handle
        if self.result_future is None: self.result_future=handle.get_result_async()
        future=self.result_future
        timeout=plan['points'][-1]['time_s']*1.5+5.
        deadline=dispatch_time+20. if getattr(self.options,'continuous_round',False) else time.monotonic()+timeout
        last=0.
        self.report['inflight_step']['status']='executing'
        self.report['inflight_step']['goal_accepted']=True
        while not future.done():
            if stop.is_set(): raise InterruptedError('执行中断')
            verify.verify_input_hashes(self.assessment)
            if self.parameter_changed: raise ValueError('执行中TCP参数被修改')
            if time.monotonic()>deadline: raise TimeoutError('执行超时')
            current=self.fresh_feedback();error=verify.pose_error(feedback_target,current['pose'])
            if error['position_mm']>self.options.position_tolerance_mm: raise ValueError('内部TCP位置跟踪偏差过大')
            if time.monotonic()-last>.1:
                samples.append(dict(feedback=current,target_pose=feedback_target,model_target_pose=plan['target']['pose'],error=error,
                                    orientation_comparison='segment_endpoint_not_time_aligned',state='executing'))
                last=time.monotonic()
            time.sleep(.02)
        response=future.result()
        if response is None or response.status!=GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val!=1:
            raise ValueError('MoveIt执行失败/被取消')
        stamp=self.fresh_feedback()['source_stamp_ns']
        deadline=min(deadline,time.monotonic()+3.) if getattr(self.options,'continuous_round',False) else time.monotonic()+3.
        while True:
            if stop.is_set(): raise InterruptedError('执行后检查被中断')
            if getattr(self.options,'continuous_round',False) and time.monotonic()>deadline:
                raise TimeoutError('整轮运动超过20秒，取消并停止后续动作')
            current=self.fresh_feedback();error=verify.pose_error(feedback_target,current['pose'])
            if current['source_stamp_ns']!=stamp and error['position_mm']<=self.options.position_tolerance_mm and error['orientation_deg']<=2.:
                break
            if time.monotonic()>deadline: raise ValueError('执行后新鲜真实TCP未达到目标容差')
            time.sleep(.03)
        self.active_goal=None;self.pending_goal=None;self.result_future=None
        self.report.pop('inflight_step',None)
        return dict(step=plan['target']['step'],status='succeeded',target_pose=feedback_target,model_target_pose=plan['target']['pose'],
                    feedback=current,error=error,samples=samples,action_status=int(response.status),
                    moveit_error_code=int(response.result.error_code.val),timestamp=datetime_iso(),motion_elapsed_s=time.monotonic()-dispatch_time)

    def cancel_active(self):
        self.cancel_requested=True
        if self.active_goal is not None and self.cancel_future is None:
            try: self.cancel_future=self.active_goal.cancel_goal_async()
            except Exception as exc: self.report['cancel_error']=str(exc)
        deadline=time.monotonic()+5.
        while self.pending_goal is not None and not self.pending_goal.done() and time.monotonic()<deadline:
            time.sleep(.02)  # Executor remains alive for delayed acceptance/cancel callback.
        if self.active_goal is not None and self.cancel_future is None:
            try: self.cancel_future=self.active_goal.cancel_goal_async()
            except Exception as exc: self.report['cancel_error']=str(exc)
        while self.cancel_future is not None and not self.cancel_future.done() and time.monotonic()<deadline:
            time.sleep(.02)
        while self.result_future is not None and not self.result_future.done() and time.monotonic()<deadline:
            time.sleep(.02)
        if self.cancel_future is not None and self.cancel_future.done():
            try:
                response=self.cancel_future.result()
                self.report['cancel_result']=dict(return_code=int(response.return_code),
                    goals_canceling=len(response.goals_canceling),scope='own_execute_trajectory_goal_only')
            except Exception as exc: self.report['cancel_error']=str(exc)
        elif self.cancel_future is not None or (self.pending_goal is not None and not self.pending_goal.done()):
            self.report['cancel_error']='goal接收/取消尚未确认；后续派发已停止，保留取消处理进程'

    def cancellation_resolved(self):
        if self.pending_goal is None: return True
        if not self.pending_goal.done(): return False
        try:
            handle=self.pending_goal.result()
            if handle is None or not handle.accepted: return True
        except Exception:
            return False  # Sending outcome is uncertain; retain cancellation ownership.
        if self.result_future is None or not self.result_future.done(): return False
        try:
            result=self.result_future.result()
            return result is not None and result.status in (4,5,6)
        except Exception:
            return False

    def close(self):
        if self.pending_goal is not None or self.active_goal is not None: self.cancel_active()
        # Never discard a late acceptance callback or an owned nonterminal action.
        # Bounded cancel attempts, persistent owner until the server returns terminal result.
        while not self.cancellation_resolved():
            self.report['cancellation_state']='unresolved_owner_retained'
            print('本工具goal终态尚未确认，保留取消处理；请现场确认停止。不要关闭本终端。',flush=True)
            if self.report.get('report_file'):
                try: cal.save_json(Path(self.report['report_file']),self.report)
                except OSError as exc:
                    print('取消状态报告写入失败，继续保留取消处理: '+str(exc),flush=True)
            self.cancel_active()
            if self.active_goal is not None and (self.result_future is None or self.result_future.done()) and not self.cancellation_resolved():
                try: self.result_future=self.active_goal.get_result_async()
                except Exception as exc: self.report['result_retry_error']=str(exc)
            time.sleep(.2)
        self.report['cancellation_state']='no_owned_nonterminal_goal'
        self.shutdown_event.set()
        if self.thread is not None: self.thread.join(timeout=2.)
        if self.executor is not None: self.executor.shutdown(timeout_sec=1.)
        if self.node is not None: self.node.destroy_node()
        if self.rclpy.ok(): self.rclpy.shutdown()


def datetime_iso():
    from datetime import datetime
    return datetime.now().astimezone().isoformat()
