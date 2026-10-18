"""Small integration surface for T5; never uses the legacy reset/home/gripper path."""
import json,queue
from .placement import PlacementStore,resolve_placement
from .settings import JsonStore
import runtime_config

class WorkbenchControl:
    def workbench_interrupted(self):
        return getattr(self,'_workbench_recover',False) or getattr(self,'_workbench_cycle_epoch',getattr(self,'_workbench_epoch',0))!=getattr(self,'_workbench_epoch',0)
    def validate_workbench_command(self,command):
        import time
        if command.get('workbench') and command.get('expires_at',float('inf'))<time.time():raise ValueError('该次派发已过期，请重新选择目标开始')
    def workbench_ack(self,command,accepted,reason=''):
        if command.get('workbench') and command.get('cycle_id'):
            from std_msgs.msg import String
            if not hasattr(self,'_workbench_ack_pub'):self._workbench_ack_pub=self.create_publisher(String,'/workbench/task_ack',10)
            self._workbench_ack_pub.publish(String(data=json.dumps(dict(cycle_id=command['cycle_id'],accepted=accepted,error=reason))))
    def _workbench_control(self,msg):
        try:
            op=json.loads(msg.data)['op']
            if op=='pause_after_cycle':self._pause_after_cycle=True
            elif op=='clear_pause':self._pause_after_cycle=False
            elif op in ('recovery_prepare','recovery_complete'):
                self._pause_after_cycle=True;self._workbench_recover=op=='recovery_prepare'
                if op=='recovery_prepare':self._workbench_epoch=getattr(self,'_workbench_epoch',0)+1
                while True:
                    try:self.cmd_queue.get_nowait()
                    except queue.Empty:break
                handle=getattr(self,'_active_goal_handle',None)
                if handle is not None:handle.cancel_goal_async()
                self._active_goal_handle=None;self._dynamic_tracker=None
                if op=='recovery_complete':
                    self._emergency_flag=False;self._emergency_req_type='';self.is_busy=False
                    # ROS ActionClient discovers the newly restarted server automatically.
            else:raise ValueError('未知工作台控制命令')
        except Exception as exc:self.get_logger().error(str(exc))
    def cache_detection(self,camera,data):
        if data.get('camera',camera)!=camera:return
        if not hasattr(self,'_detections_by_camera'):self._detections_by_camera={}
        self._detections_by_camera[camera]=data
        if camera!=getattr(self,'_active_camera','d455'):return
        self._latest_detection=data
        if getattr(self,'_dynamic_tracker',None) is not None:self._dynamic_tracker.observe(data)
    def placement_snapshot(self,class_name,camera):
        if not hasattr(self,'_placement_store'):self._placement_store=PlacementStore(runtime_config.workspace_root()/'config/workbench/placements.json')
        data=self._placement_store.get()
        if self._placement_store.error:self.get_logger().error(self._placement_store.error)
        return resolve_placement(class_name,camera,data)
    def _wait_control_future(self,future):
        """Break an old action wait when its T1 is being replaced; normal algorithms unchanged."""
        epoch=getattr(self,'_workbench_epoch',0)
        while not future.done():
            if self.workbench_interrupted() or epoch!=getattr(self,'_workbench_epoch',0):
                def cancel_late(f):
                    if not f.cancelled():
                        handle=f.result()
                        if hasattr(handle,'cancel_goal_async'):handle.cancel_goal_async()
                future.add_done_callback(cancel_late)
                return False
            import rclpy
            if not rclpy.ok():return False
            rclpy.spin_once(self,timeout_sec=.01)
        return not future.cancelled()
