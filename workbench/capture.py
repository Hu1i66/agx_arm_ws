"""Raw images only. Latest frame consumed once, independent camera timestamps."""
import datetime,json,pathlib,threading,time,uuid,zipfile
import cv2
from .settings import atomic_json

class Capture:
    def __init__(self,root,emit=None,metadata=None):
        self.root=pathlib.Path(root);self.emit=emit or (lambda *a:None);self.lock=threading.RLock()
        self.condition=threading.Condition(self.lock);self.generation=0
        self.metadata=metadata or (lambda:{})
        self.frames={};self.used={};self.session=None;self.records=[];self.count=0;self.stop_event=threading.Event();self.thread=None
    def update(self,camera,pixels,stamp):
        with self.condition:
            self.frames[camera]=(pixels.copy(),stamp);self.condition.notify_all()
    def _session(self):
        if self.session is None:
            self.session=self.root/(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')+'_'+uuid.uuid4().hex[:6]);self.session.mkdir(parents=True)
    def once(self,cameras):
        with self.lock:
            self._session();errors={}
            for camera in cameras:
                frame=self.frames.get(camera)
                if frame is None or self.used.get(camera)==frame[1]:errors[camera]='等待新的原始图像';continue
                pixels,stamp=frame;folder=self.session/camera;folder.mkdir(exist_ok=True)
                target=folder/f'{self.count+1:06d}_{stamp:.9f}.png'
                if not cv2.imwrite(str(target),pixels):raise OSError('PNG 写入失败: '+str(target))
                self.used[camera]=stamp;self.count+=1
                self.records.append(dict(camera=camera,stamp_s=stamp,path=str(target.relative_to(self.session)),width=pixels.shape[1],height=pixels.shape[0],configuration=self.metadata()))
            atomic_json(self.session/'session.json',dict(kind='raw_rgb_dataset',images=self.records))
            result=dict(count=self.count,directory=str(self.session),errors=errors,images=self.records[-12:])
            self.emit('capture.progress',result);return result
    def next_frame(self,cameras,timeout=2.):
        if not cameras:raise ValueError('请选择至少一台相机')
        with self.condition:
            baseline={c:self.frames.get(c,(None,None))[1] for c in cameras};generation=self.generation;deadline=time.monotonic()+timeout
            while generation==self.generation and time.monotonic()<deadline:
                if all(c in self.frames and self.frames[c][1]!=baseline[c] for c in cameras):break
                self.condition.wait(max(0.,deadline-time.monotonic()))
            if generation!=self.generation:return {'state':'cancelled','count':self.count}
            fresh=[c for c in cameras if c in self.frames and self.frames[c][1]!=baseline[c]]
            result=self.once(fresh)
            result['errors'].update({c:'等待下一张原始图像' for c in cameras if c not in fresh})
            return result

    def start(self,cameras,interval=2.):
        self.stop();interval=float(interval)
        if interval<=0:raise ValueError('采样间隔必须大于零')
        self.stop_event.clear()
        def loop():
            while not self.stop_event.is_set():
                started=time.monotonic()
                try:self.next_frame(cameras,timeout=min(.5,interval))
                except Exception as exc:self.emit('capture.progress',{'error':str(exc)})
                self.stop_event.wait(max(0.,interval-(time.monotonic()-started)))
        self.thread=threading.Thread(target=loop,daemon=True);self.thread.start()
        return {'state':'capturing','interval_s':interval}
    def stop(self):
        self.stop_event.set()
        with self.condition:self.generation+=1;self.condition.notify_all()
        if self.thread and self.thread is not threading.current_thread():self.thread.join()
        return {'state':'stopped','count':self.count}
    def export(self):
        with self.lock:
            if self.session is None:raise ValueError('当前没有可导出的采集会话')
            path=self.session.with_suffix('.zip')
            with zipfile.ZipFile(path,'w',compression=zipfile.ZIP_STORED) as archive:
                for file in self.session.rglob('*'):
                    if file.is_file():archive.write(file,file.relative_to(self.session))
            return path
