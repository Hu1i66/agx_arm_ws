"""Existing launcher adapter. Ordinary ensure and forced arm recovery are separate."""
import pathlib
import shlex
import subprocess
import threading
import runtime_launcher

COMPONENTS = [('can','d455'),('activate','d455'),('arm','d455'),
              ('camera','d455'),('camera','dabai'),('camera_tf','d455'),
              ('camera_tf','dabai'),('detector','dual'),('action','d455'),('conveyor','d455')]

class Components:
    def __init__(self, root, launcher=runtime_launcher, run=None, control=None, emit=None, conveyor=None):
        self.root=pathlib.Path(root); self.launcher=launcher
        self.run=run or self._run; self.control=control or (lambda op:None)
        self.emit=emit or (lambda *args:None); self.conveyor=conveyor
        self.cancel=threading.Event(); self.lock=threading.Lock()
        self.completed=set()

    def command(self, component):
        if component=='can':
            script=self.root/'src/agx_arm_ros/scripts/can_activate.sh'
            # pkexec provides the normal system authentication dialog; no saved password.
            return ['pkexec','bash','-c',f'bash {shlex.quote(str(script))} can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0']
        return ['/usr/bin/python3',str(self.root/'activate_robot.py')]

    def _run(self, argv):
        self.emit('log.batch',{'source':'连接命令','lines':['$ '+shlex.join(argv)]})
        proc=subprocess.Popen(argv,cwd=self.root,env=self.launcher.build_env('dual'),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        lines=[]
        for line in proc.stdout:
            lines.append(line); self.emit('log.batch',{'source':'连接命令','lines':[line.rstrip()]})
        return {'returncode':proc.wait(),'output':''.join(lines),'argv':argv}

    def start(self, component, camera='d455', force=False):
        if component in ('can','activate'):
            if component in self.completed and not force:
                return {'result':'reused','detail':'本次服务已执行；使能状态仍以实际反馈为准'}
            result=self.run(self.command(component))
            if result['returncode']: raise RuntimeError(f'{component} 执行失败: '+result.get('output',''))
            self.completed.add(component); return result
        if component=='conveyor':
            if self.conveyor is None: raise RuntimeError('传送带接口不可用')
            return self.conveyor('connect')
        result=self.launcher.ensure_background(component,camera=camera,initiator='workbench',wait_ready=False)
        if result.get('result') not in ('spawned','reused','reused_external'):
            raise RuntimeError(result.get('detail',str(result)))
        self.emit('component.status',dict(component=component,camera_id=camera,**result))
        return result

    def stop(self, component, camera='d455'):
        if component=='conveyor': return self.conveyor('disconnect')
        if component in ('can','activate'): return {'detail':'一次性操作，无常驻进程'}
        return self.launcher.stop_component(component,camera=camera,initiator='workbench',allow_cross_initiator=True)

    def ensure_missing_components(self):
        self.cancel.clear(); results=[]
        # Skip activation when registered arm already lives; PID is not an enable-health claim.
        arm_live=False
        if hasattr(self.launcher,'_manager'):
            mgr=self.launcher._manager(); rec=mgr.get_record('arm')
            arm_live=bool(rec and mgr.verify_identity(rec)[0])
        if not arm_live and hasattr(self.launcher,'_node_exists'):
            arm_live=self.launcher._node_exists('/agx_arm_ctrl_single_node')
        for component,camera in COMPONENTS:
            if self.cancel.is_set(): break
            try:
                if component in ('can','activate') and arm_live:
                    r={'result':'reused','detail':'机械臂进程已在运行；如连接异常请手动恢复'}
                else: r=self.start(component,camera)
                results.append(dict(component=component,camera_id=camera,**r))
            except Exception as exc:
                results.append(dict(component=component,camera_id=camera,error=str(exc)))
            self.emit('component.status',results[-1])
        return results

    def recover_arm_connection(self):
        if not self.lock.acquire(blocking=False): raise RuntimeError('恢复正在执行')
        try:
            steps=[('暂停派发、停带并取消旧目标',lambda:self.control('recovery_prepare')),
                   ('退出旧机械臂 / MoveIt',lambda:self.stop('arm')),
                   ('重新激活 CAN',lambda:self.start('can',force=True)),
                   ('重新激活机械臂',lambda:self.start('activate',force=True)),
                   ('按当前 TCP 启动机械臂 / MoveIt',lambda:self.start('arm')),
                   ('刷新动作状态并保持暂停',lambda:self.control('recovery_complete'))]
            for i,(label,fn) in enumerate(steps):
                self.emit('arm.recovery.progress',{'step':i,'label':label,'state':'running'})
                result=fn()
                if i==1 and result.get('result')=='not_registered' and hasattr(self.launcher,'_node_exists') and self.launcher._node_exists('/agx_arm_ctrl_single_node'):
                    raise RuntimeError('旧T1由外部终端运行，无法确认工作空间所有权；请停止该终端后重试恢复')
                if i==1 and result.get('result') not in ('stopped','not_registered','already_exited','stale_record_removed'):
                    raise RuntimeError('旧 T1 尚未退出: '+str(result))
                if i==4 and result.get('result')!='spawned':raise RuntimeError('T1未创建新实例，请检查外部机械臂进程: '+str(result))
                self.emit('arm.recovery.progress',{'step':i,'label':label,'state':'completed','result':result})
            return {'state':'awaiting_feedback','detail':'连接流程已完成，使能待实际反馈确认；抓取保持暂停'}
        except Exception as exc:
            self.emit('arm.recovery.progress',{'state':'failed','error':str(exc)})
            raise
        finally: self.lock.release()
