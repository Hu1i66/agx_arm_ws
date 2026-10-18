"""Calibration-only transforms, without ROS or robot motion."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from runtime_geometry import transform, inverse, tcp_to_flange, camera_in_base, graspnet_tcp_target


def test_full_tcp_inverse_nonzero_xyz_rpy():
    S=transform([.01,-.02,.15],Rotation.from_euler('xyz',[.2,-.3,.4]).as_quat())
    target=transform([.4,.1,.2],Rotation.from_euler('xyz',[-.5,.6,.2]).as_quat())
    np.testing.assert_allclose(tcp_to_flange(target,S)@S,target,atol=1e-12)


def test_dabai_flange_calibration_survives_new_tcp():
    S=transform([.01,0,.15],[0,0,0,1]);F=transform([.3,0,.2],Rotation.from_euler('y',.4).as_quat())
    C=transform([-.067,.004,.046],[0,0,0,1])
    np.testing.assert_allclose(camera_in_base(C,'link6',F@S,S),F@C,atol=1e-12)
    np.testing.assert_allclose(camera_in_base(C,'base_link',None,S),C)


def test_graspnet_depth_is_applied_once_at_fingertip():
    C=transform([.3,0,.5],Rotation.from_euler('x',np.pi).as_quat())
    S=transform([.01,-.005,.148],Rotation.from_euler('xyz',[.1,.2,.3]).as_quat())
    R=Rotation.from_euler('y',np.pi/2).as_matrix()
    T=graspnet_tcp_target(C,[0,0,.2],R,.03,S)
    expected=(C@np.r_[(np.array([0,0,.2])+.03*R[:,0]),1])[:3]
    np.testing.assert_allclose(T[:3,3],expected)
    np.testing.assert_allclose(T[:3,:3],C[:3,:3]@R[:,[1,2,0]]@S[:3,:3])
    np.testing.assert_allclose(tcp_to_flange(T,S)@S,T,atol=1e-12)


def test_bad_camera_parent_and_missing_depth_rejected():
    with pytest.raises(ValueError):camera_in_base(np.eye(4),'unknown',np.eye(4),np.eye(4))
    with pytest.raises((ValueError,TypeError)):graspnet_tcp_target(np.eye(4),[0,0,0],np.eye(3),None,np.eye(4))


def test_dabai_mount_uses_flange_calibration_and_preserves_robot(monkeypatch):
    from runtime_geometry import add_dabai_mount
    import runtime_config as rc
    monkeypatch.setenv('AGX_ARM_WS',str(__import__('pathlib').Path(__file__).resolve().parents[1]))
    import xml.etree.ElementTree as ET
    urdf='<robot name="piper"><link name="link6"/><link name="tcp_link"/></robot>'
    srdf='<robot name="piper"><group name="arm"/></robot>'
    model,semantic=add_dabai_mount(urdf,srdf)
    root=ET.fromstring(model)
    joint=root.find("joint[@name='dabai_mount_joint']")
    assert joint.find('parent').get('link')=='link6'
    he=rc.load_handeye('dabai');C=transform(he['position'],he['orientation'])
    body=root.find("link[@name='dabai_mount_body']/collision[@name='dabai_body']/origin")
    np.testing.assert_allclose(np.fromstring(body.get('xyz'),sep=' '),(C@np.array([0,0,-.025,1]))[:3])
    assert root.find("link[@name='tcp_link']") is not None
    pairs=ET.fromstring(semantic).findall('disable_collisions')
    assert [(x.get('link1'),x.get('link2')) for x in pairs]==[('link6','dabai_mount_body')]


def test_rgbd_centroid_is_inside_object_not_visible_surface():
    from runtime_geometry import object_centroid_from_rgbd
    # Downward camera: table at base z=0, object top at z=.06m.
    depth=np.full((80,100),1000,dtype=np.uint16)
    depth[28:52,37:63]=940
    C=transform([0,0,1],Rotation.from_euler('x',np.pi).as_quat())
    result=object_centroid_from_rgbd(depth,[35,26,65,54],[200,200,50,40],C)
    assert result['centroid'][2]==pytest.approx(.03,abs=.001)
    assert result['surface_position'][2]==pytest.approx(.06,abs=.001)
    assert result['centroid'][0]==pytest.approx(0,abs=.005)
    with pytest.raises(ValueError):object_centroid_from_rgbd(np.zeros_like(depth),[35,26,65,54],[200,200,50,40],C)
