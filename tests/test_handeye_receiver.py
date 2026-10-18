"""Offline ROS-import checks; no node, camera or robot is started."""
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def test_camera_qos_and_interrupted_shutdown():
    code=r'''
from types import SimpleNamespace as NS
from unittest.mock import patch
from rclpy.qos import ReliabilityPolicy as R
from handeye_calibration_ros.camera_qos import image_subscription_qos
from handeye_calibration_ros import handeye_calibration as he

def fake(mode,publishers,camera="d455"):
    return NS(declare_parameter=lambda *a:None,get_parameter=lambda name:NS(value=mode if name=="image_reliability" else camera),
        get_publishers_info_by_topic=lambda topic:[NS(qos_profile=NS(reliability=r)) for r in publishers],
        get_logger=lambda:NS(info=lambda *a:None,warning=lambda *a:None))
for offered,expected in [([R.RELIABLE],R.RELIABLE),([R.BEST_EFFORT],R.BEST_EFFORT),
                         ([R.RELIABLE,R.BEST_EFFORT],R.BEST_EFFORT)]:
    q=image_subscription_qos(fake("auto",offered),"/image")
    assert q.reliability==expected and q.depth==1
assert image_subscription_qos(fake("best_effort",[R.RELIABLE]),"/image").reliability==R.BEST_EFFORT
assert image_subscription_qos(fake("reliable",[]),"/image").reliability==R.RELIABLE
try:image_subscription_qos(fake("typo",[]),"/image")
except ValueError:pass
else:raise AssertionError("invalid QoS mode accepted")
# SIGINT may have already closed rclpy; main must not shut it down twice.
events=[]
node=NS(run=lambda:(_ for _ in ()).throw(KeyboardInterrupt()),
        stop_spin=lambda:events.append("stop"),destroy_node=lambda:events.append("destroy"))
with patch.object(he.rclpy,"init"),patch.object(he,"HandEyeCalibrationNode",return_value=node),\
     patch.object(he.rclpy,"ok",return_value=False),patch.object(he.rclpy,"shutdown") as shutdown:
    he.main();shutdown.assert_not_called()
assert events==["stop","destroy"]
print("receiver_qos_and_shutdown_ok")
'''
    result=subprocess.run(['bash','-c',
        'source /opt/ros/humble/setup.bash && export PYTHONDONTWRITEBYTECODE=1 && '
        'export PYTHONPATH="$1/src/handeye_calibration_ros:$PYTHONPATH" && exec /usr/bin/python3 -c "$2"',
        'test',str(ROOT),code],capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'receiver_qos_and_shutdown_ok' in result.stdout
