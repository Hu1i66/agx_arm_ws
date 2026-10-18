"""Real Pinocchio FK/IK round trip, without constructing a ROS node."""
import os
from pathlib import Path
import subprocess


def test_actual_ik_uses_tcp_position_with_nonzero_xyz_rpy():
    root=Path(__file__).resolve().parents[1]
    script=r'''
import numpy as np
from scipy.spatial.transform import Rotation
import auto_sorting_action as action
import runtime_config as rc
from runtime_geometry import transform
solver=action.PinocchioIKSolver(str(rc.resolve_resource('ik_urdf')))
pin=action._pin
joints=np.array([0.,.5,-.8,.1,.6,-.2])
for S in [solver.tcp_transform,transform([.012,-.018,.15],Rotation.from_euler('xyz',[.2,-.3,.4]).as_quat())]:
    solver.tcp_transform=S
    q=pin.neutral(solver.model)
    for i,index in enumerate(solver.joint_q_indices):q[index]=joints[i]
    pin.forwardKinematics(solver.model,solver.data,q);pin.updateFramePlacements(solver.model,solver.data)
    flange=solver.data.oMf[solver.ee_frame_id]
    F=np.eye(4);F[:3,:3]=flange.rotation;F[:3,3]=flange.translation
    target=F@S
    ok,solution,error,_=solver.get_ik_solution(target[:3,3],Rotation.from_matrix(target[:3,:3]).as_quat(),initial_guess=joints)
    assert ok and error<1e-8,(ok,error)
    print('offline TCP FK/IK round trip passed',error)
'''
    env=dict(os.environ,AGX_ARM_WS=str(root),PYTHONDONTWRITEBYTECODE='1')
    result=subprocess.run(['bash','-c','source /opt/ros/humble/setup.bash && source install/setup.bash && exec /usr/bin/python3 -c "$1"','ik-check',script],
                          cwd=root,env=env,text=True,capture_output=True,timeout=60)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    assert result.stdout.count('offline TCP FK/IK round trip passed')==2
