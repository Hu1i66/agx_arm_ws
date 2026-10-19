"""Targeted ROS IO integration in an isolated domain; no drivers or motion servers."""
import json,threading,time,types
import pytest


def test_real_ros_raw_images_sources_and_actual_feedback(tmp_path):
    rclpy=pytest.importorskip('rclpy')
    from sensor_msgs.msg import Image,JointState
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    from workbench.ros_adapter import RosAdapter
    from workbench.capture import Capture
    received={};events=[]
    service=types.SimpleNamespace(capture=Capture(tmp_path),direction_samples=None,
        tasks=types.SimpleNamespace(detection=lambda camera,data:received.update({camera:data})),
        emit=lambda *x:events.append(x),logs=types.SimpleNamespace(add=lambda *x:None),log_ros=lambda *x:None)
    rclpy.init(domain_id=223);adapter=RosAdapter(service);publisher=rclpy.create_node('workbench_io_fixture')
    def wait(predicate):
        end=time.monotonic()+3
        while not predicate() and time.monotonic()<end:time.sleep(.01)
        assert predicate()
    try:
        pubs=[]
        for camera,topic in [('d455','/camera/camera/color/image_raw'),('dabai','/dabai/color/image_raw')]:
            pub=publisher.create_publisher(Image,topic,qos_profile_sensor_data);pubs.append(pub)
            wait(lambda:pub.get_subscription_count()>0)
            pub.publish(Image(width=4,height=3,encoding='bgr8',step=12,data=[27]*36))
        wait(lambda:len(service.capture.frames)==2)
        assert service.capture.frames['dabai'][0].shape==(3,4,3)
        assert (service.capture.frames['d455'][0]==27).all()
        control=publisher.create_publisher(JointState,'/control/joint_states',10)
        control.publish(JointState(name=['joint'+str(i) for i in range(1,7)],position=[.9]*6));time.sleep(.05)
        assert adapter.joints is None
        feedback=publisher.create_publisher(JointState,'/feedback/joint_states',10);wait(lambda:feedback.get_subscription_count()>0)
        feedback.publish(JointState(name=['joint'+str(i) for i in range(1,7)],position=[.1]*6))
        wait(lambda:adapter.joints is not None)
        assert adapter.capture_actual_pose()['joints_rad']==[.1]*6
        for camera,topic in [('d455','/detection_info'),('dabai','/dabai/detection_info')]:
            pub=publisher.create_publisher(String,topic,10);pubs.append(pub);wait(lambda:pub.get_subscription_count()>0)
            pub.publish(String(data=json.dumps({'camera':camera,'motion_session':camera,'objects':[{'track_id':1}]})))
        wait(lambda:len(received)==2)
        assert received['d455']['motion_session']!=received['dabai']['motion_session']
    finally:
        adapter._executor.shutdown();adapter.thread.join(timeout=2);adapter.destroy_node();publisher.destroy_node();rclpy.shutdown()


def test_current_camera_rate_expires_after_stream_stops(monkeypatch):
    import workbench.ros_adapter as module
    adapter=module.RosAdapter.__new__(module.RosAdapter)
    adapter.rates={};adapter.rate_times={}
    now=[1.0]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    for stamp in (1.0,1.5,2.0):
        now[0]=stamp;adapter._mark_rate('d455')
    now[0]=2.0
    assert adapter.current_rate('d455')==pytest.approx(2.0)
    now[0]=5.0
    assert adapter.current_rate('d455')==0.
