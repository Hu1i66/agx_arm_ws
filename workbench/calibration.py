"""PTY wrapper for onsite tools; selected numerical results keep their original quality."""
import codecs,datetime,json,math,os,pathlib,pty,signal,subprocess,threading,time,uuid
import yaml
from .settings import atomic_json
import runtime_launcher as launcher

class PtySession:
    def __init__(self,argv,cwd,env,on_output):
        self.master,slave=pty.openpty();self.closed=False;self.output=on_output
        self.proc=subprocess.Popen(argv,cwd=cwd,env=env,stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
        os.close(slave);self.thread=threading.Thread(target=self._read,daemon=True);self.thread.start()
    def _read(self):
        decoder=codecs.getincrementaldecoder('utf-8')('replace')
        try:
            while True:
                data=os.read(self.master,8192)
                if not data:break
                self.output(decoder.decode(data))
        except OSError:pass
        finally:self.output(decoder.decode(b'',final=True))
    def write(self,text):
        if self.proc.poll() is not None:raise ValueError('采集程序已退出，请查看输出')
        os.write(self.master,text.encode())
    def stop(self):
        if self.proc.poll() is None:os.killpg(self.proc.pid,signal.SIGINT)
    def close(self):
        if not self.closed:os.close(self.master);self.closed=True

class Calibration:
    def __init__(self,root,emit=None):self.root=pathlib.Path(root);self.emit=emit or (lambda *x:None);self.sessions={};self.lock=threading.Lock()
    def start_calibration(self,kind,camera_id='d455',options=None):
        options=options or {};sid=uuid.uuid4().hex;directory=self.root/'data/calibration'/camera_id/kind/(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')+'_'+sid[:6]);directory.mkdir(parents=True)
        if kind in ('intrinsics','handeye','tcp'):
            extra=[]
            if kind=='handeye':extra=['-p',f'result_save_path:={directory}']
            if kind=='tcp':extra=['-p',f'result_save_path:={directory}']
            argv=launcher.build_command('calibrate-'+kind,camera=camera_id,session_dir=directory,ros_args=extra)
        elif kind=='verify-tcp':
            argv=launcher.build_command('verify-tcp',camera=camera_id,ros_args=['--workbench','--mode',options.get('mode','plan'),'--candidate',str(self.root/'config/calibration/tcp.json'),'--sweep','--angle-deg',str(options.get('angle_deg',5.)),'--steps',str(options.get('steps',5)),'--speed',str(options.get('speed',.03)),'--axes',*options.get('axes',['x','y']),'--output',str(directory/'report.json')])
        elif kind=='verify-handeye':
            # Existing TF/model loaded by RViz, not a fabricated scene screenshot.
            argv=['rviz2','-d',str(self.root/'config/workbench/calibration_verify.rviz')]
        else:raise ValueError('未知标定类型')
        def output(text):
            record=self.sessions.get(sid);pid=record['proc'].proc.pid if record else None
            self.emit('log.batch',{'source':'标定:'+sid[:6],'session':sid,'pid':pid,'lines':[text]})
        proc=PtySession(argv,self.root,launcher.build_env('dual'),output)
        self.sessions[sid]=dict(proc=proc,kind=kind,camera_id=camera_id,directory=directory,started_at=time.time(),seen={})
        return {'session_id':sid,'directory':str(directory),'pid':proc.proc.pid,'argv':argv}
    def send_calibration_input(self,session_id,action):
        session=self.sessions[session_id]
        if action=='stop':session['proc'].stop()
        else:session['proc'].write({'capture':'\n','undo':'d\n','solve':'q\n','finish':'c\n'}[action])
        return {'sent':action}
    def stop_motion_verifications(self):
        for session in list(self.sessions.values()):
            if session['kind']=='verify-tcp':
                try:session['proc'].stop()
                except ProcessLookupError:pass
        return {'state':'verification_stop_requested'}
    def poll(self):
        for sid,s in list(self.sessions.items()):
            code=s['proc'].proc.poll()
            if code is not None and not s.get('exit_reported') and not s['proc'].thread.is_alive():
                s['exit_reported']=True;s['proc'].close();self.emit('calibration.progress',{'session_id':sid,'state':'exited','returncode':code,'directory':str(s['directory'])})
            for path in s['directory'].rglob('*'):
                if path.suffix not in ('.json','.yaml'):continue
                mtime=path.stat().st_mtime
                if s['seen'].get(str(path))==mtime:continue
                try:
                    raw=path.read_text();data=yaml.safe_load(raw) if path.suffix=='.yaml' else json.loads(raw)
                    s['seen'][str(path)]=mtime
                    rows=data.get('samples',data.get('frames',[]));count=len(rows) if isinstance(rows,list) else rows
                    self.emit('calibration.progress' if path.name=='frames.json' else 'calibration.result',dict(session_id=sid,path=str(path),sample_count=count,data=data))
                except (ValueError,OSError):continue
    def list_files(self):
        rows=[]
        for base in [self.root/'data/calibration',self.root/'config/calibration']:
            for path in base.rglob('*'):
                if path.suffix not in ('.json','.yaml') or 'backup' in str(path):continue
                try:
                    data=yaml.safe_load(path.read_text())
                    if not isinstance(data,dict):continue
                    if not any(k in data for k in ('camera_matrix','position','tcp_offset')):continue
                    kind='intrinsics' if 'camera_matrix' in data else 'tcp' if 'tcp_offset' in data else 'handeye'
                    meta=data.get('metadata',data.get('candidate',{})) or {}
                    camera=meta.get('camera_id', 'd455' if 'd455' in str(path) else 'dabai' if 'dabai' in str(path) else None)
                    rows.append(dict(path=str(path),kind=kind,camera_id=camera,modified=path.stat().st_mtime,data=data))
                except (ValueError,OSError,yaml.YAMLError):continue
        return sorted(rows,key=lambda r:r['modified'],reverse=True)
    def apply_calibration(self,kind,camera_id,source_path):
        if kind=='tcp':camera_id='d455' # one public TCP, independent of camera selector
        source=pathlib.Path(source_path).expanduser().resolve();raw=source.read_bytes();data=yaml.safe_load(raw)
        if not isinstance(data,dict):raise ValueError('标定文件必须为对象')
        def finite(values,n):
            if not isinstance(values,list) or len(values)!=n or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):raise ValueError('标定数值不完整')
        meta=data.get('metadata',data.get('candidate',{})) or {}
        if camera_id not in ('d455','dabai'):raise ValueError('未知相机')
        if meta.get('units',data.get('units','m'))!='m':raise ValueError('标定长度单位须为 m')
        if meta.get('camera_id',camera_id)!=camera_id and kind!='tcp':raise ValueError('候选所属相机与选择不一致')
        if kind=='intrinsics':
            finite(data['camera_matrix']['data'],9)
            distortion=data['distortion_coefficients']['data'];finite(distortion,len(distortion))
            if len(distortion) not in (4,5,8,12,14):raise ValueError('不支持的畸变系数数量')
            if data.get('image_width',0)<=0 or data.get('image_height',0)<=0:raise ValueError('内参须记录图像宽高')
            if data['camera_matrix']['data'][6:]!=[0,0,1]:raise ValueError('内参矩阵最后一行须为 [0,0,1]')
            if data['camera_matrix']['data'][0]<=0 or data['camera_matrix']['data'][4]<=0:raise ValueError('内参焦距必须大于零')
            target=self.root/f'config/calibration/{camera_id}/intrinsics.yaml';affected=[('detector','dual')]
        elif kind=='handeye':
            finite(data['position'],3);finite(data['orientation'],4)
            expected={'mode':'eye_to_hand' if camera_id=='d455' else 'eye_in_hand','transform_parent':'base_link' if camera_id=='d455' else meta.get('robot_pose_frame','link6'),'transform_child':'camera_color_optical_frame' if camera_id=='d455' else 'dabai_color_optical_frame','quaternion_order':'xyzw'}
            if camera_id=='dabai' and expected['transform_parent'] not in ('link6','tcp_link'):raise ValueError('Dabai 父坐标系须为 link6 或 tcp_link')
            if camera_id=='dabai' and expected['transform_parent']=='tcp_link':finite(meta.get('tcp_offset'),6)
            for key,wanted in expected.items():
                if meta.get(key,data.get(key,wanted))!=wanted:raise ValueError('标定坐标约定不一致: '+key)
            if sum(v*v for v in data['orientation'])<1e-12:raise ValueError('四元数不能为零')
            target=self.root/('config/calibration/d455_eye_to_hand.json' if camera_id=='d455' else 'config/calibration/dabai_eye_in_hand.json');affected=[('detector','dual'),('camera_tf',camera_id)]
            if camera_id=='dabai':affected += [('arm','d455'),('workbench','d455')] # wrist collision body uses this calibration
        elif kind=='tcp':
            finite(data.get('tcp_offset'),6)
            if any(abs(v)>math.pi*2 for v in data['tcp_offset'][3:]):raise ValueError('TCP 姿态须为弧度')
            target=self.root/'config/calibration/tcp.json';affected=[('arm','d455'),('workbench','d455')]
        else:raise ValueError('未知标定文件类型')
        # Copy bytes verbatim, never approve or rewrite original quality fields.
        with self.lock:
            cal=self.root/'config/calibration';manifest_path=cal/'manifest.json';manifest=json.loads(manifest_path.read_text())
            backup=cal/'workbench_backup'/(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')+'_'+uuid.uuid4().hex[:8]);backup.mkdir(parents=True)
            (backup/'manifest.json').write_bytes(manifest_path.read_bytes())
            record=dict(backup=str(backup),kind=kind,camera_id=camera_id,source=str(source),target=str(target.relative_to(self.root)),selected_at=time.time(),state='pending_restart',affected=affected,original_quality=meta.get('quality_status',data.get('quality_status')))
            if target.exists():(backup/'active').write_bytes(target.read_bytes())
            atomic_json(backup/'selection.json',record)
            target.parent.mkdir(parents=True,exist_ok=True);temp=target.with_suffix(target.suffix+'.tmp');temp.write_bytes(raw);os.replace(temp,target)
            manifest.setdefault('workbench_selections',{})[kind+':'+camera_id]=record
            try:atomic_json(manifest_path,manifest)
            except Exception:
                if (backup/'active').exists():target.write_bytes((backup/'active').read_bytes())
                else:target.unlink()
                raise
        result=dict(record,backup=str(backup));self.emit('settings.applied',result);return result
    def selection_state(self,evidence):
        """Compare live loaded numbers, never infer effectiveness from a PID/restart."""
        import numpy as np
        from scipy.spatial.transform import Rotation
        def close(a,b):
            a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
            return a.shape==b.shape and np.isfinite(a).all() and np.allclose(a,b,atol=1e-8,rtol=0)
        def pose(p,q,p2,q2):
            return close(p,p2) and close(Rotation.from_quat(q).as_matrix(),Rotation.from_quat(q2).as_matrix())
        try:records=json.loads((self.root/'config/calibration/manifest.json').read_text()).get('workbench_selections',{})
        except (OSError,ValueError):return {}
        result={}
        for key,row in records.items():
            actual=evidence.get(row['camera_id'] if row['kind']!='tcp' else 'tcp',{})
            ok=False;reason='等待组件加载后的实际数值反馈'
            try:
                data=yaml.safe_load((self.root/row['target']).read_text())
                if time.time()-actual.get('stamp',0)<3:
                    if row['kind']=='intrinsics':ok=close(data['camera_matrix']['data'],actual.get('k',[])) and close(data['distortion_coefficients']['data'],actual.get('d',[]))
                    elif row['kind']=='handeye':ok=pose(data['position'],data['orientation'],actual.get('position',[]),actual.get('orientation',[])) and pose(data['position'],data['orientation'],actual.get('tf_position',[]),actual.get('tf_orientation',[]))
                    else:
                        wanted=data['tcp_offset'];ok=all(close(wanted,actual.get(name,[])) for name in ('sdk_start','sdk_parameter','moveit','workbench'))
                    reason='实际加载数值一致' if ok else '实际加载数值不同或反馈不完整，请重载受影响组件'
            except (ValueError,KeyError,OSError,TypeError):reason='文件或实际反馈尚不完整'
            result[key]=dict(row,state='effective' if ok else 'pending_restart',detail=reason)
        return result

    def restore(self,backup):
        folder=pathlib.Path(backup);record=json.loads((folder/'selection.json').read_text());target=self.root/record['target']
        if not (folder/'active').exists():raise ValueError('该版本没有上一个活动副本')
        with self.lock:
            target.write_bytes((folder/'active').read_bytes())
            manifest_path=self.root/'config/calibration/manifest.json';current=json.loads(manifest_path.read_text());previous=json.loads((folder/'manifest.json').read_text());key=record['kind']+':'+record['camera_id']
            old=previous.get('workbench_selections',{}).get(key)
            if old:current.setdefault('workbench_selections',{})[key]=old
            else:current.setdefault('workbench_selections',{}).pop(key,None)
            atomic_json(manifest_path,current)
        return {'state':'pending_restart','affected':record['affected']}


def selected_file(root,kind,camera,path):
    """Explicit current selection, not a fabricated approval/quality flag."""
    try:
        root=pathlib.Path(root);manifest=json.loads((root/'config/calibration/manifest.json').read_text())
        row=manifest.get('workbench_selections',{}).get(kind+':'+camera,{})
        return (root/row.get('target','')).resolve()==pathlib.Path(path).resolve() and bool(row.get('source'))
    except (OSError,ValueError):return False
