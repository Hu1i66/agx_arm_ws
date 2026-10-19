"""Persistent, workspace-local NDJSON service. Starting it never enables hardware."""
import concurrent.futures,collections,fcntl,json,os,pathlib,re,socketserver,threading,time,traceback
from .settings import JsonStore,atomic_json
from .placement import PlacementStore
from .capture import Capture
from .calibration import Calibration
from .components import Components,COMPONENTS
from .logs import Logs,ReconnectNotice
from .tasks import Tasks
import runtime_config as rc
import runtime_launcher as launcher

def usb_link_speed(vendor,product,root=pathlib.Path('/sys/bus/usb/devices')):
    """Return the negotiated Mbps for a connected USB device, if visible."""
    for device in root.iterdir():
        try:
            if (device/'idVendor').read_text().strip()!=vendor:continue
            if (device/'idProduct').read_text().strip()!=product:continue
            return float((device/'speed').read_text().strip())
        except (OSError,ValueError):continue
    return None

class Workbench:
    def __init__(self,root,with_ros=True):
        self.root=pathlib.Path(root);self.clients=set();self.clients_lock=threading.Lock()
        self.logs=Logs(self.root/'log/workbench');self.notice=ReconnectNotice();self.last_log_poll=0.
        self.capture=Capture(self.root/'data/datasets',self.emit,metadata=self.capture_sources)
        self.settings=JsonStore(self.root/'config/workbench/settings.json',{'schema_version':1,'manual':{'velocity':.2,'acceleration':.2},'static':{'approach_height_m':.06,'gripper_force_n':1.5,'gripper_settle_s':.05},'camera':{},'vision':{}})
        self.saved_poses=JsonStore(rc.resolve_resource('saved_poses'),{})
        self.placements=PlacementStore(self.root/'config/workbench/placements.json')
        self.ros=None;self.belt=None;self.belt_commanded=None;self.belt_lock=threading.RLock()
        self.tasks=Tasks(self.send,self.control,self.conveyor,self.emit)
        self.tasks.receiver_alive=lambda:self.ros is not None and self.ros.count_subscribers('/sorting_cmds')>0
        self.calibration=Calibration(self.root,self.emit)
        self.components=Components(self.root,control=self.tasks.recovery,emit=self.emit,conveyor=self.conveyor)
        if with_ros:
            from .ros_adapter import RosAdapter
            self.ros=RosAdapter(self)
        self.log_offsets={};self.pending_logs=collections.deque();self.external_started=time.time();self.direction_samples=None
    def capture_sources(self):
        paths=['config/calibration/manifest.json','config/workbench/settings.json','config/dynamic_grasp.json']
        result={}
        for path in paths:
            try:result[path]=json.loads((self.root/path).read_text())
            except (OSError,ValueError):result[path]={'error':'来源文件不可读'}
        return result
    def emit(self,event,data):
        if event=='log.batch':
            for line in data.get('lines',[]):self.logs.add(data.get('source','工作台'),line,pid=data.get('pid'),session=data.get('session'))
        payload=(json.dumps({'event':event,'data':data},ensure_ascii=False,default=str)+'\n').encode()
        with self.clients_lock:clients=list(self.clients)
        for client in clients:
            try:client.send(payload)
            except OSError:pass
    def send(self,topic,data):
        if self.ros is None:raise RuntimeError('ROS 接口不可用')
        self.ros.send(topic,data)
    def control(self,op):self.send('/workbench/control',{'op':op})
    def conveyor(self,op):
        with self.belt_lock:
            if self.belt is None:
                if op in ('stop','disconnect'):return {'connected':False,'detail':'传送带未连接'}
                from stm32_conveyor import STM32Conveyor
                self.belt=STM32Conveyor()
            if op in ('start','stop') and self.belt_commanded==(op=='start') and self.belt.is_connected:return self.belt.get_status()
            result=getattr(self.belt,op)()
            if op in ('start','stop'):
                if result is False:raise RuntimeError('传送带命令未确认: '+str(self.belt.get_status()))
                self.belt_commanded=op=='start'
            return dict(self.belt.get_status(),command_result=result)
    def status(self):
        mgr=launcher._manager();rows=[]
        for component,camera in COMPONENTS:
            key=launcher.component_key(component,camera=camera);rec=mgr.get_record(key)
            live=bool(rec and mgr.verify_identity(rec)[0]);row=dict(component=component,camera_id=camera,state='running' if live else 'not_started',record=rec)
            if component in ('can','activate'):row.update(state='command_completed' if component in self.components.completed else 'unknown',command=self.components.command(component))
            rows.append(row)
        now=time.time();seen=self.ros.seen if self.ros else {}
        cameras={c:dict(state='receiving' if now-seen.get(c,0)<2 else 'waiting',age_s=now-seen[c] if c in seen else None,input_fps=self.ros.current_rate(c) if self.ros else 0,detection_fps=self.ros.current_rate('yolo:'+c) if self.ros else 0,detection_age_s=now-seen['yolo:'+c] if 'yolo:'+c in seen else None) for c in ('d455','dabai')}
        cameras['d455']['usb_speed_mbps']=usb_link_speed('8086','0b5c')
        return dict(config_errors=[e for e in (self.settings.error,self.placements.error) if e],components=rows,task=self.tasks.status(),placements=self.placements.get(),parameters=self.settings.get(),
            cameras=cameras,
            feedback=self.ros.feedback_snapshot() if self.ros else None,
            reconnect=dict(count=self.notice.count,last_at=self.notice.last),conveyor=self.belt.get_status() if self.belt else {'connected':False},
            nodes=self.ros.get_node_names() if self.ros else [],socket=str(self.root/'.runtime/workbench.sock'))
    def log_ros(self,source,text,stamp):
        managed={'agx_arm_ctrl_single_node':'arm','piper_moveit_action_client':'action','object_detector':'detector:dual'}.get(source)
        record=launcher._manager().get_record(managed) if managed else None
        if not record or not record.get('log_file') or not launcher._manager().verify_identity(record)[0]:
            row=self.logs.add(source,text,ros_stamp=stamp);self.pending_logs.append(row)
        if source=='agx_arm_ctrl_single_node':
            if self.notice.instance is None and self.ros:
                infos=self.ros.get_publishers_info_by_topic('/rosout')
                ids=[bytes(x.endpoint_gid).hex() for x in infos if x.node_name=='agx_arm_ctrl_single_node']
                if ids:self.notice.set_instance('external:'+','.join(ids),self.external_started)
            if 'not enabled' in text or 'teaching' in text.lower():self.emit('arm.control_reason',{'detail':'机械臂未使能' if 'not enabled' in text else '机械臂处于示教模式','original':text})
            notice=self.notice.feed(text,self.notice.instance,stamp)
            if notice:self.emit('arm.reconnect_required',notice)
    def apply_parameter_group(self,group,values):
        data=self.settings.get()
        if group=='dynamic':
            original=json.loads((self.root/'config/dynamic_grasp.json').read_text())
            if set(values)!=set(original):raise ValueError('动态参数字段须与现有文件一致')
            import math
            if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values.values()):raise ValueError('参数必须为有限数值')
            for key,value in values.items():
                if value<=0 and key not in ('intercept_s_pref_m','intercept_s_max_m'):raise ValueError(key+' 必须大于零（沿用原动态配置要求）')
                if type(original[key]) is int:
                    if not float(value).is_integer():raise ValueError(key+' 必须为整数')
                    values[key]=int(value)
            atomic_json(self.root/'config/dynamic_grasp.json',values);result={'state':'next_object','values':values}
        elif group=='vision':
            result=self.ros.parameters('/object_detector',values);data[group].update(values);self.settings.save(data)
        elif group.startswith('camera:'):
            result=self.ros.camera_exposure(group.split(':')[1],values);data['camera'][group.split(':')[1]]=values;self.settings.save(data)
        elif group in ('manual','static'):
            import math
            if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values.values()):raise ValueError('参数必须为有限数值')
            if group=='manual' and any(not 0<v<=1 for v in values.values()):raise ValueError('手动速度与加速度比例应在0到1之间')
            if group=='static':
                if not .5<=values.get('gripper_force_n',1.5)<=3.:raise ValueError('夹持力沿用0.5到3N范围')
                if any(not 0<values.get(k,.65)<=1 for k in ('joint_velocity_scaling','joint_acceleration_scaling')):raise ValueError('关节速度与加速度比例应在0到1之间')
                if any(values.get(k,0)<0 for k in ('approach_height_m','soft_lift_m','gripper_settle_s')):raise ValueError('高度与时间不能为负数')
            data[group]=values;self.settings.save(data);result={'state':'next_plan' if group=='manual' else 'next_object','values':values}
        else:raise ValueError('未知参数组')
        self.emit('settings.applied',dict(group=group,**result));return result
    def dispatch(self,op,args):
        if op=='status.get':return self.status()
        if op=='tools.conveyor_calibration':
            if self.direction_samples is None:
                self.direction_samples={};return {'state':'collecting','detail':'开始记录D455轨迹；让物体随传送带移动，再次点击结束并计算方向'}
            from conveyor_calib import fit_belt_direction,estimate_belt_speed_vision
            tracks=self.direction_samples;self.direction_samples=None
            if not tracks:raise ValueError('没有采集到有效D455目标轨迹')
            rows=max(tracks.values(),key=len);dx,dy,info=fit_belt_direction([(r[1],r[2]) for r in rows])
            result=dict(info,dx=dx,dy=dy,vision_speed_mps=estimate_belt_speed_vision(rows))
            path=rc.resolve_resource('conveyor_calib')
            if path.exists():path.with_suffix('.json.previous').write_bytes(path.read_bytes())
            atomic_json(path,result);return dict(result,state='saved',detail='方向标定已保存；现有动作节点重启后加载')

        if op=='system.start_missing':return self.components.ensure_missing_components()
        if op=='system.cancel':self.components.cancel.set();return {'state':'cancel_requested'}
        if op=='system.stop_all':
            warnings=[]
            try:self.capture.stop()
            except Exception as exc:warnings.append(dict(component='capture',error=str(exc)))
            for sid,session in list(self.calibration.sessions.items()):
                try:session['proc'].stop()
                except ProcessLookupError:pass
                except Exception as exc:warnings.append(dict(component='calibration:'+sid,error=str(exc)))
            return self.components.close_all_connections()+warnings
        if op=='arm.recover':return self.components.recover_arm_connection()
        if op.startswith('component.'):
            # An explicit activation click must run again after USB re-enumeration.
            if op=='component.start' and args['component'] in ('can','activate'):
                return self.components.start(args['component'],args.get('camera_id','d455'),force=True)
            return getattr(self.components,op.split('.')[1])(args['component'],args.get('camera_id','d455'))
        if op=='motion.start':return self.tasks.start(**args)
        if op=='motion.send':
            if args.get('pick',{}).get('_reference_frame')=='legacy_link6':raise ValueError('旧法兰坐标缺少朝向，请从实际TCP反馈重新保存')
            cmd=args.get('cmd');topic='/emergency_stop' if cmd in ('emergency_stop','standby','reset') else '/sorting_cmds'
            if topic=='/emergency_stop':args=dict(args,req='estop' if cmd=='emergency_stop' else cmd)
            if cmd=='emergency_stop':
                self.tasks.pause=True;self.tasks.continuous=False;self.tasks.queue_running=False
                self.send(topic,args) # independent arm stop must precede serial IO
                self.calibration.stop_motion_verifications()
                try:self.conveyor('stop')
                except Exception as exc:self.logs.add('急停传送带',str(exc));return {'state':'arm_stop_sent','conveyor_error':str(exc)}
                return {'state':'sent'}
            self.send(topic,args);return {'state':'sent'}
        if op in ('motion.pause_after_cycle','session.close'):return self.tasks.request_pause_after_cycle()
        if op=='conveyor.send':return self.conveyor(args['action'])
        if op=='poses.capture':return self.ros.capture_actual_pose()
        if op=='poses.list':return self.saved_poses.get()
        if op=='poses.save':
            pose=self.ros.capture_actual_pose();tcp=pose['tcp_xyzw']
            if tcp is None:raise ValueError('等待实际TCP反馈')
            data=self.saved_poses.get();data[args['name']]=dict(zip(('x','y','z','qx','qy','qz','qw'),tcp),_reference_frame='tcp_link');return self.saved_poses.save(data)
        if op=='poses.delete':
            data=self.saved_poses.get();data.pop(args['name'],None);return self.saved_poses.save(data)
        if op=='queue.get':return self.tasks.queued
        if op=='queue.set':
            for row in args['tasks']:
                if not isinstance(row,dict) or row.get('cmd')!='sort' or not row.get('pick'):raise ValueError('队列必须由原sort命令组成')
            self.tasks.queued=list(args['tasks']);return self.tasks.status()
        if op=='queue.start':
            self.tasks.pause=False;self.tasks.continuous=False;self.tasks.queue_running=True;self.control('clear_pause');return self.tasks.status()
        if op=='queue.clear':self.tasks.queued.clear();self.tasks.queue_running=False;return self.tasks.status()

        if op=='placements.save':return self.placements.save(args['data'])
        if op=='parameters.get':
            import yaml
            saved=self.settings.get();defaults=yaml.safe_load((self.root/'config/vision/fruit.yaml').read_text())['object_detector']['ros__parameters']
            vision={c+'.'+k:defaults.get(c+'.'+k,default) for c in ('d455','dabai') for k,default in [('yolo_conf',.25),('yolo_iou',.7),('detect_period_s',defaults.get('detect_period_s',.1))]}
            vision.update(saved.get('vision',{}));saved['vision']=vision
            return dict(saved,dynamic=json.loads((self.root/'config/dynamic_grasp.json').read_text()))
        if op=='parameters.apply':return self.apply_parameter_group(args['group'],args['values'])
        if op=='calibration.start':return self.calibration.start_calibration(args['kind'],args.get('camera_id','d455'),args.get('options'))
        if op=='calibration.input':return self.calibration.send_calibration_input(args['session_id'],args['action'])
        if op=='calibration.list':
            evidence=self.ros.calibration_evidence() if self.ros else {}
            return dict(files=self.calibration.list_files(),selections=self.calibration.selection_state(evidence))
        if op=='calibration.apply':return self.calibration.apply_calibration(args['kind'],args.get('camera_id','d455'),args['source_path'])
        if op=='calibration.restore':return self.calibration.restore(args['backup'])
        if op=='calibration.restart':
            results=[]
            for component,camera in args['affected']:
                if component=='workbench':results.append({'component':component,'state':'reopen_required','detail':'重新打开窗口以加载新 TCP 模型'});continue
                stopped=self.components.stop(component,camera)
                if stopped.get('result') not in ('stopped','not_registered','already_exited','stale_record_removed'):raise RuntimeError(str(stopped))
                results.append(self.components.start(component,camera))
            return {'state':'restarted_awaiting_feedback','results':results}
        if op=='capture.once':return self.capture.next_frame(args.get('cameras',['d455','dabai']))
        if op=='capture.start':return self.capture.start(args.get('cameras',['d455','dabai']),args.get('interval',2.))
        if op=='capture.stop':return self.capture.stop()
        if op=='capture.export':return {'path':str(self.capture.export())}
        if op=='logs.subscribe':return self.logs.query(**args)[-10000:]
        if op=='logs.export':return {'path':str(self.logs.export(**args))}
        if op=='logs.history':
            path=pathlib.Path(args['path']);return {'text':path.read_text(errors='replace')}
        if op=='diagnostics.list':return [str(x) for x in sorted((self.root/'data/diagnostics/dynamic_grasp').glob('*.json'),reverse=True)]
        if op=='diagnostics.read':return json.loads(pathlib.Path(args['path']).read_text())
        raise ValueError('未知操作: '+op)
    def poll(self):
        self.calibration.poll()
        mgr=launcher._manager();record=mgr.get_record('arm');instance=record.get('record_id') if record else None
        if instance!=self.notice.instance and (instance is not None or not str(self.notice.instance).startswith('external:')):
            self.notice.set_instance(instance,record.get('started_at',time.time()) if record else time.time())
        # Read incremental registered logs, with each new T1's log offset (never replay an old instance as a new warning).
        registry=mgr.read_registry()
        records=registry.get('components',registry) if isinstance(registry,dict) else {}
        batch=[]
        for key,rec in records.items():
            if not isinstance(rec,dict) or not rec.get('log_file'):continue
            path=pathlib.Path(rec['log_file'])
            if str(path) not in self.logs.offsets:self.logs.offsets[str(path)]=(int(rec.get('log_offset',path.stat().st_size if path.exists() else 0)),b'')
            for line in self.logs.tail(path):
                batch.append(self.logs.add(key,line,instance=rec.get('record_id'),pid=rec.get('pid')))
                if key=='arm' and rec.get('record_id')==self.notice.instance:
                    if '[agx_arm_ctrl_single_node]' in line and 'Agx_arm is not connected, cannot control' in line:
                        stamp_match=re.search(r'\[(\d+\.\d+)\]',line)
                        stamp=float(stamp_match[1]) if stamp_match else time.time()
                        notice=self.notice.feed('Agx_arm is not connected, cannot control',rec['record_id'],stamp)
                        if notice:self.emit('arm.reconnect_required',notice)
        while self.pending_logs:batch.append(self.pending_logs.popleft())
        if batch:self.emit('logs.rows',batch)
        try:self.tasks.tick()
        except Exception as exc:
            self.tasks.pause=True;self.tasks.continuous=False;self.conveyor('stop');self.emit('task.status',dict(self.tasks.status(),error=str(exc)))
        if self.belt:
            s=self.belt.get_status();self.send('/conveyor_status',dict(s,type='conveyor_status',commanded_running=self.belt_commanded,status_age_s=max(0.,time.time()-s['last_status_time'])))

class Handler(socketserver.StreamRequestHandler):
    def setup(self):
        super().setup();self.lock=threading.Lock();self.control_lock=threading.Lock();self.closed=False
    def send(self,payload):
        with self.lock:
            if not self.closed:self.request.sendall(payload)
    def handle(self):
        service=self.server.service
        with service.clients_lock:service.clients.add(self)
        try:
            for line in self.rfile:
                try:request=json.loads(line)
                except ValueError:continue
                def execute(req=request):
                    try:
                        if req['op'].startswith(('motion.','queue.')) or req['op']=='session.close':
                            with self.control_lock:
                                if self.closed and req.get('args',{}).get('cmd')!='emergency_stop':raise RuntimeError('窗口已断开，未派发的任务已取消')
                                result=service.dispatch(req['op'],req.get('args',{}))
                        else:result=service.dispatch(req['op'],req.get('args',{}))
                        response={'id':req.get('id'),'ok':True,'result':result}
                    except Exception as exc:
                        detail=traceback.format_exc();service.logs.add('工作台',detail)
                        response={'id':req.get('id'),'ok':False,'error':{'message':str(exc),'detail':detail}}
                    try:self.send((json.dumps(response,ensure_ascii=False,default=str)+'\n').encode())
                    except OSError:pass
                self.server.pool.submit(execute)
        finally:
            self.closed=True
            with service.clients_lock:service.clients.discard(self)
            # Detached service, never stop arm/camera/action processes on UI disconnect.
            try:
                with self.control_lock:service.tasks.request_pause_after_cycle()
            except Exception as exc:service.logs.add('窗口断开',str(exc))

class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads=True

def main():
    root=rc.workspace_root();runtime=root/'.runtime';runtime.mkdir(exist_ok=True)
    lock=(runtime/'workbench-service.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:return
    path=runtime/'workbench.sock'
    if path.exists():path.unlink()
    import rclpy
    rclpy.init();service=Workbench(root)
    with Server(str(path),Handler) as server, concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        os.chmod(path,0o600);server.service=service;server.pool=pool
        def poll():
            while True:
                try:service.poll()
                except Exception:service.logs.add('服务轮询',traceback.format_exc())
                time.sleep(.25)
        threading.Thread(target=poll,daemon=True).start();server.serve_forever(poll_interval=.25)

if __name__=='__main__':main()
