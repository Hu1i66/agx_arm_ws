"""UI-independent dispatch using the existing sorting commands and conveyor timings."""
import threading,time,uuid

class Tasks:
    def __init__(self,send,control,conveyor,emit):
        self.send=send;self.control=control;self.conveyor=conveyor;self.emit=emit
        self.lock=threading.RLock();self.mode='d455-static';self.continuous=False
        self.pause=True;self.inflight=None;self.detections={};self.phase='paused';self.stopped_at=0.;self.failed=set();self.abandon_at=0.;self.queued=[];self.queue_running=False
    def detection(self,camera,data):
        with self.lock:self.detections[camera]=(data,time.time())
    def select(self,camera,track_id,session=None):
        data,at=self.detections.get(camera,({},0))
        if time.time()-at>1.:raise ValueError('等待当前相机的新检测结果')
        if session and session!=data.get('motion_session'):raise ValueError('目标所属视觉会话已变化')
        obj=next((o for o in data.get('objects',[]) if o.get('track_id')==track_id),None)
        if obj is None:raise ValueError('当前目标已不在画面中')
        if not obj.get('base_position_m'):raise ValueError('该目标尚无有效基座坐标')
        return data,obj
    def dispatch(self,camera,track_id,session=None,dynamic=False):
        data,obj=self.select(camera,track_id,session)
        bp=obj['base_position_m'];name=obj.get('object_name','object')
        command=dict(cmd='sort_dynamic' if dynamic else 'sort',camera_id=camera,target_track_id=track_id,
            motion_session=data.get('motion_session'),vision_target=True,detection_stamp=data.get('header_stamp',time.time()),
            object_name=name,pick_name=name,pick=dict(bp,qx=1.,qy=0.,qz=0.,qw=0.),
            grasp_geometry=obj.get('grasp_geometry'),object_diameter_m=obj.get('estimated_diameter_m'),
            cycle_id=uuid.uuid4().hex,workbench=True,start_intent=True)
        self.inflight=command['cycle_id'];self.send('/sorting_cmds',command);self.phase='grasping';return command
    def start(self,mode,continuous=False,track_id=None,session=None):
        with self.lock:
            self.mode=mode;self.pause=False;self.continuous=bool(continuous);self.failed.clear()
            self.control('clear_pause');self.phase='waiting'
            camera='dabai' if mode=='dabai-static' else 'd455'
            if continuous and camera=='dabai':raise ValueError('Dabai 使用所选目标单次抓取')
            if not continuous:return self.dispatch(camera,track_id,session)
            self.conveyor('start' if mode=='d455-dynamic' else 'stop')
            return self.status()
    def request_pause_after_cycle(self):
        with self.lock:
            self.pause=True;self.continuous=False;self.queue_running=False;self.control('pause_after_cycle')
            if not self.inflight:self.conveyor('stop');self.phase='paused'
            else:self.phase='pause_pending'
            return self.status()
    def recovery(self,op):
        with self.lock:
            self.pause=True;self.continuous=False;self.queue_running=False;self.queued.clear();self.inflight=None;self.phase='paused';self.failed.clear()
            self.conveyor('stop');self.control(op)
    def result(self,result):
        with self.lock:
            if not self.inflight:return
            if result.get('cycle_id') and result['cycle_id']!=self.inflight:return
            self.inflight=None;self.failed.update(result.get('failed_track_ids',[]))
            if result.get('ok') is False:self.pause=True;self.continuous=False
            if self.pause or not (self.continuous or self.queue_running):self.conveyor('stop');self.phase='paused'
            else:self.phase='waiting'
            self.emit('task.status',dict(self.status(),result=result))
    def tick(self):
        with self.lock:
            if self.pause or self.inflight:return
            if self.queue_running:
                if not self.queued:self.queue_running=False;self.pause=True;self.phase='paused';return
                command=dict(self.queued.pop(0),cycle_id=uuid.uuid4().hex,workbench=True,start_intent=True)
                if command.get('pick',{}).get('_reference_frame')=='legacy_link6':raise ValueError('旧法兰坐标缺少朝向，请从实际TCP反馈重新保存该点')
                self.inflight=command['cycle_id'];self.send('/sorting_cmds',command);self.phase='grasping';return
            if not self.continuous:return
            data,at=self.detections.get('d455',({},0))
            if time.time()-at>1.:return
            dynamic=self.mode=='d455-dynamic';line=130 if dynamic else 260
            candidates=[o for o in data.get('objects',[]) if o.get('base_position_m') and o.get('bbox_pixel',{}).get('x2',-1)>=line and o.get('track_id') not in self.failed]
            obj=max(candidates,key=lambda o:o['bbox_pixel']['x1']+o['bbox_pixel']['x2'],default=None)
            if dynamic:
                far=[o for o in candidates if o['bbox_pixel']['x2']>=490]
                if far and time.time()-self.abandon_at>=2.5:
                    self.conveyor('stop')
                    if any(o['bbox_pixel']['x2']>=510 for o in far):self.abandon_at=time.time();return
                    self.dispatch('d455',far[0]['track_id'],data.get('motion_session'),dynamic=False);return
                self.conveyor('start')
                if obj:self.dispatch('d455',obj['track_id'],data.get('motion_session'),dynamic=True)
            elif obj:
                if self.phase!='settling':self.conveyor('stop');self.stopped_at=time.time();self.phase='settling';return
                if time.time()-self.stopped_at>=1.:self.dispatch('d455',obj['track_id'],data.get('motion_session'))
            else:self.conveyor('start');self.phase='waiting'
    def status(self):return dict(mode=self.mode,continuous=self.continuous,paused=self.pause,inflight=self.inflight,phase=self.phase,queue_size=len(self.queued),queue_running=self.queue_running)
