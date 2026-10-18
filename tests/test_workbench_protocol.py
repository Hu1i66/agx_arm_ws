import concurrent.futures,json,socket,threading,types,uuid
from workbench.service import Server,Handler

def test_protocol_chinese_errors_and_disconnect_pause():
    paused=threading.Event()
    service=types.SimpleNamespace(clients=set(),clients_lock=threading.Lock(),
        tasks=types.SimpleNamespace(request_pause_after_cycle=paused.set),logs=types.SimpleNamespace(add=lambda *x:None))
    def dispatch(op,args):
        if op=='failure':raise ValueError('真实错误详情')
        return {'中文':'工作台','args':args}
    service.dispatch=dispatch
    address='\0workbench-test-'+uuid.uuid4().hex
    with Server(address,Handler) as server,concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        server.service=service;server.pool=pool
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        client=socket.socket(socket.AF_UNIX);client.settimeout(2);client.connect(address)
        stream=client.makefile('rb')
        client.sendall(b'not-json\n'+json.dumps({'id':'1','op':'status.get','args':{'名称':'示教'}}).encode()+b'\n')
        row=json.loads(stream.readline());assert row['ok'] and row['result']['中文']=='工作台'
        client.sendall(b'{"id":"2","op":"failure"}\n');row=json.loads(stream.readline())
        assert row['id']=='2' and not row['ok'] and '真实错误详情' in row['error']['detail']
        stream.close();client.close();assert paused.wait(2)
        server.shutdown();thread.join()
