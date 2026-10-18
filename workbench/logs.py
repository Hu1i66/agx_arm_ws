import collections,datetime,json,pathlib,re,threading,time

class ReconnectNotice:
    def __init__(self):self.set_instance(None,float('inf'))
    def set_instance(self,instance,started_at):self.instance=instance;self.started_at=started_at;self.count=0;self.last=None
    def feed(self,text,instance,stamp):
        if instance!=self.instance or instance is None or stamp<self.started_at:return None
        if text!='Agx_arm is not connected, cannot control':return None
        if self.last is not None and abs(self.last-stamp)<1e-6:return None
        self.count+=1;self.last=stamp
        return dict(count=self.count,last_at=stamp,instance=instance,detail='机械臂连接异常，建议执行恢复；原因待确认')

class Logs:
    def __init__(self,root):
        self.root=pathlib.Path(root);self.root.mkdir(parents=True,exist_ok=True)
        self.path=self.root/('workbench-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.jsonl')
        self.rows=collections.deque(maxlen=10000);self.lock=threading.Lock();self.offsets={}
    def add(self,source,text,**metadata):
        row=dict(time=time.time(),source=source,level='ERROR' if re.search(r'ERROR|Traceback|失败',text) else 'WARN' if 'WARN' in text else 'INFO',text=text,**metadata)
        with self.lock:
            self.rows.append(row)
            if self.path.exists() and self.path.stat().st_size>50*1024*1024:self.path=self.path.with_name('workbench-'+str(time.time_ns())+'.jsonl')
            with self.path.open('a',encoding='utf-8') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
        return row
    def query(self,source='',keyword='',level='',since=0):
        with self.lock:return [r for r in self.rows if (not source or source in r['source']) and keyword.lower() in r['text'].lower() and (not level or r['level']==level) and r['time']>=since]
    def export(self,**filters):
        path=self.root/f'export-{time.time_ns()}.log';source=filters.get('source','');keyword=filters.get('keyword','').lower();level=filters.get('level','');since=filters.get('since',0)
        with path.open('w',encoding='utf-8') as out:
            for file in sorted(self.root.glob('workbench-*.jsonl')):
                with file.open(encoding='utf-8',errors='replace') as stream:
                    for line in stream:
                        try:r=json.loads(line)
                        except ValueError:continue
                        if source and source not in r['source'] or keyword not in r['text'].lower() or level and r['level']!=level or r['time']<since:continue
                        out.write(f"{r['time']:.3f} [{r['source']}] [{r['level']}] {r['text']}\n")
        return path
    def tail(self,path):
        path=pathlib.Path(path)
        if not path.is_file():return []
        key=str(path);offset,partial=self.offsets.get(key,(0,b''))
        if path.stat().st_size<offset:offset=0;partial=b''
        with path.open('rb') as f:f.seek(offset);data=partial+f.read();offset=f.tell()
        parts=data.split(b'\n');self.offsets[key]=(offset,parts[-1])
        return [x.decode('utf-8',errors='replace') for x in parts[:-1]]
