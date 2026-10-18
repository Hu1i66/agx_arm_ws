"""Shared calibrated SE(3) geometry. No ROS imports and no motion side effects."""
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
import runtime_config as rc


def transform(position, quaternion):
    p=np.asarray(position,dtype=float);q=np.asarray(quaternion,dtype=float)
    if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('Invalid rigid-transform position/quaternion')
    T=np.eye(4);T[:3,:3]=Rotation.from_quat(q).as_matrix();T[:3,3]=p
    return T


def inverse(T):
    T=np.asarray(T,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('Invalid rigid transform')
    out=np.eye(4);out[:3,:3]=T[:3,:3].T;out[:3,3]=-out[:3,:3]@T[:3,3]
    return out


def load_tcp():
    path=rc.workspace_root()/'config/calibration/tcp.json'
    data=json.loads(path.read_text())
    if data.get('quality_status')!='passed' or data.get('approved') is not True:
        raise ValueError('Active TCP is not accepted: '+str(path))
    offset=np.asarray(data['tcp_offset'],dtype=float)
    if offset.shape!=(6,) or not np.isfinite(offset).all():raise ValueError('Invalid active tcp_offset')
    return offset.tolist()


def tcp_transform():
    offset=load_tcp()
    return transform(offset[:3],Rotation.from_euler('xyz',offset[3:]).as_quat())


def tcp_to_flange(target,S):
    return np.asarray(target)@inverse(S)


def camera_in_base(calibration,parent,tcp_pose,S):
    if parent=='base_link':return np.asarray(calibration).copy()
    if tcp_pose is None:raise ValueError('Dabai requires image-time robot pose')
    if parent=='link6':return tcp_to_flange(tcp_pose,S)@calibration
    if parent=='tcp_link':return np.asarray(tcp_pose)@calibration
    raise ValueError('Unsupported camera parent: '+str(parent))


def graspnet_tcp_target(camera_pose,translation,rotation,depth,S):
    depth=float(depth)
    if not np.isfinite(depth) or not 0<=depth<=.1:raise ValueError('Invalid GraspNet depth')
    R=np.asarray(rotation,dtype=float);p=np.asarray(translation,dtype=float)
    if R.shape!=(3,3) or p.shape!=(3,) or not np.isfinite(R).all() or not np.isfinite(p).all():
        raise ValueError('Invalid GraspNet pose')
    T=np.eye(4)
    T[:3,3]=(camera_pose@np.r_[p+depth*R[:,0],1])[:3]
    T[:3,:3]=camera_pose[:3,:3]@R[:,[1,2,0]]@S[:3,:3]
    return T


def assert_robot_tcp(node):
    """Read-only check: SDK parameter and MoveIt fixed joint must match the active TCP."""
    import xml.etree.ElementTree as ET
    import rclpy
    from rcl_interfaces.srv import GetParameters
    wanted=tcp_transform()
    from handeye_calibration_ros.tcp_verification_ros import startup_offset
    started=startup_offset(rc.workspace_root())
    start_values=np.asarray(started['offset'],dtype=float)
    if start_values.shape!=(6,) or not np.allclose(transform(start_values[:3],Rotation.from_euler('xyz',start_values[3:]).as_quat()),wanted,atol=1e-8,rtol=0):
        raise ValueError('T1启动时SDK的TCP与活动配置不一致；在线param set不能代替重启')
    for name,parameter in [('/agx_arm_ctrl_single_node','tcp_offset'),('/move_group','robot_description')]:
        client=node.create_client(GetParameters,name+'/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=2.):raise ValueError('T1/MoveIt unavailable: '+name)
            request=GetParameters.Request();request.names=[parameter]
            future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=3.)
            if not future.done() or future.exception():raise ValueError('TCP parameter check timed out')
            values=future.result().values
            if len(values)!=1:raise ValueError('Missing '+parameter)
            value=values[0]
            if parameter=='tcp_offset':
                if value.type!=8:raise ValueError('tcp_offset is not a double array')
                x=np.asarray(value.double_array_value,dtype=float)
                if x.shape!=(6,):raise ValueError('Invalid driver TCP')
                actual=transform(x[:3],Rotation.from_euler('xyz',x[3:]).as_quat())
            else:
                if value.type!=4:raise ValueError('Missing MoveIt robot model')
                model=ET.fromstring(value.string_value)
                mount_cfg=json.loads((rc.workspace_root()/'config/robot/dabai_mount.json').read_text())
                if mount_cfg['enabled']:
                    expected,_=add_dabai_mount('<robot><link name="link6"/></robot>','<robot/>')
                    expected_link=ET.fromstring(expected).find("link[@name='dabai_mount_body']")
                    actual_link=model.find("link[@name='dabai_mount_body']")
                    if actual_link is None:raise ValueError('运行中的T1缺少Dabai碰撞体，请重启新版T1')
                    for collision in expected_link.findall('collision'):
                        actual_shape=actual_link.find("collision[@name='"+collision.get('name')+"']")
                        if actual_shape is None:raise ValueError('Dabai碰撞形状缺失')
                        for element,attr in [('origin','xyz'),('origin','rpy'),('geometry/box','size')]:
                            wanted_numbers=np.fromstring(collision.find(element).get(attr),sep=' ')
                            actual_element=actual_shape.find(element)
                            if actual_element is None:raise ValueError('Dabai碰撞模型不完整')
                            actual_numbers=np.fromstring(actual_element.get(attr,''),sep=' ')
                            if actual_numbers.shape!=wanted_numbers.shape or not np.allclose(actual_numbers,wanted_numbers,atol=1e-8,rtol=0):
                                raise ValueError('Dabai碰撞模型已变更，请重启T1')
                joint=next((j for j in model.findall('joint') if j.find('child') is not None and j.find('child').get('link')=='tcp_link'),None)
                if joint is None or joint.get('type')!='fixed' or joint.find('parent').get('link')!='link6':raise ValueError('MoveIt TCP is not fixed to link6')
                origin=joint.find('origin')
                xyz=list(map(float,origin.get('xyz','0 0 0').split())) if origin is not None else [0.]*3
                rpy=list(map(float,origin.get('rpy','0 0 0').split())) if origin is not None else [0.]*3
                actual=transform(xyz,Rotation.from_euler('xyz',rpy).as_quat())
            if not np.allclose(actual,wanted,atol=1e-8,rtol=0):raise ValueError('运行中的T1/MoveIt未使用活动TCP；停止旧T1并用新参数启动，禁止在线param set代替')
        finally:node.destroy_client(client)


def object_centroid_from_rgbd(depth_mm, bbox, intrinsics, camera_pose, *, distortion=None):
    """Estimate a supported object's extent centre, rather than its visible top.

    The support plane is fitted to the ring outside the detection box. Its hidden
    bottom and measured top bound the estimated centre; no class-size/Z bias is used.
    """
    from scipy.ndimage import label
    depth=np.asarray(depth_mm)
    fx,fy,cx,cy=map(float,intrinsics)
    if depth.ndim!=2 or not np.isfinite([fx,fy,cx,cy]).all() or min(fx,fy)<=0:
        raise ValueError('Invalid RGB-D intrinsics/depth')
    h,w=depth.shape
    x1,y1,x2,y2=map(int,bbox)
    x1=max(0,x1);y1=max(0,y1);x2=min(w,x2);y2=min(h,y2)
    if x2-x1<3 or y2-y1<3:raise ValueError('Object ROI too small')
    pad=max(12,int(max(x2-x1,y2-y1)*.3))
    a,b=max(0,x1-pad),max(0,y1-pad);c,d=min(w,x2+pad),min(h,y2+pad)
    yy,xx=np.mgrid[b:d,a:c]
    z=depth[b:d,a:c].astype(float)*.001
    valid=np.isfinite(z)&(z>.1)&(z<3.)
    if distortion is not None and np.any(np.asarray(distortion)):
        import cv2
        k=np.array([[fx,0.,cx],[0.,fy,cy],[0.,0.,1.]])
        pixels=np.stack([xx,yy],axis=-1).astype(float).reshape(-1,1,2)
        rays=cv2.undistortPoints(pixels,k,np.asarray(distortion,dtype=float)).reshape(*z.shape,2)
        points=np.concatenate([rays*z[...,None],z[...,None]],axis=-1)
    else:
        points=np.stack([(xx-cx)*z/fx,(yy-cy)*z/fy,z],axis=-1)
    points=points@camera_pose[:3,:3].T+camera_pose[:3,3]
    inside=(xx>=x1)&(xx<x2)&(yy>=y1)&(yy<y2)
    ring=points[valid&~inside]
    if len(ring)<30:raise ValueError('No measured support plane around object')
    ring=ring[::max(1,len(ring)//1200)]
    rng=np.random.default_rng(0);best=None;count=0
    for _ in range(60):
        tri=ring[rng.choice(len(ring),3,replace=False)]
        normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);norm=np.linalg.norm(normal)
        if norm<1e-9:continue
        normal/=norm
        if abs(normal[2])<.85:continue
        normal*=np.sign(normal[2]);offset=-normal@tri[0]
        mask=np.abs(ring@normal+offset)<.004
        if mask.sum()>count:best=(normal,offset,mask);count=int(mask.sum())
    if best is None or count<max(30,.3*len(ring)):raise ValueError('Support plane fit unavailable')
    # Refine the measured plane with its inliers.
    inliers=ring[best[2]]
    coeff=np.linalg.lstsq(np.c_[inliers[:,:2],np.ones(len(inliers))],inliers[:,2],rcond=None)[0]
    plane_z=points[:,:,0]*coeff[0]+points[:,:,1]*coeff[1]+coeff[2]
    height=points[:,:,2]-plane_z
    mask=valid&inside&(height>.004)&(height<.3)
    components,n=label(mask)
    if not n:raise ValueError('No object above support plane')
    sizes=np.bincount(components.ravel());sizes[0]=0
    selected=components==int(np.argmax(sizes))
    if selected.sum()<30:raise ValueError('Too few object points for centroid')
    obj=points[selected];object_height=float(np.percentile(height[selected],95))
    xy=np.mean(np.percentile(obj[:,:2],[5,95],axis=0),axis=0)
    support_z=float(coeff[0]*xy[0]+coeff[1]*xy[1]+coeff[2])
    centroid=np.r_[xy,support_z+object_height*.5]
    # 在已经去除台面的同一批物体点上估计长短轴，不再对原bbox混入背景做PCA。
    _, axes = np.linalg.eigh(np.cov(obj[:, :2].T))
    short_axis, long_axis = axes[:, 0], axes[:, 1]
    width = float(np.ptp(np.percentile(obj[:, :2] @ short_axis, [5, 95])))
    length = float(np.ptp(np.percentile(obj[:, :2] @ long_axis, [5, 95])))
    grasp_geometry = dict(frame='base_link', short_axis_base=[*map(float, short_axis), 0.0],
                          width_m=width, length_m=length,
                          elongation_ratio=length / max(width, 1e-9))
    return dict(centroid=centroid,surface_position=np.r_[xy,support_z+object_height],
                support_z=support_z,height_m=object_height,point_count=int(len(obj)),
                method='rgbd_extent_center_on_measured_support_plane', grasp_geometry=grasp_geometry)


def short_axis_tool_rotation(short_axis, approach_axis):
    """Piper两指沿工具Y闭合；保持其XY投影沿物体短轴，工具Z沿接近方向。"""
    z = np.asarray(approach_axis, dtype=float)
    z = z / np.linalg.norm(z)
    short = np.asarray(short_axis, dtype=float)[:2]
    y = np.r_[short, -np.dot(short, z[:2]) / z[2]]
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z])


def add_dabai_mount(robot_xml, semantic_xml):
    """Add an explicitly approximate, configurable camera/bracket collision body."""
    import xml.etree.ElementTree as ET
    cfg=json.loads((rc.workspace_root()/'config/robot/dabai_mount.json').read_text())
    if not cfg['enabled']:return robot_xml,semantic_xml
    he=rc.load_handeye('dabai')
    if he['parent']!='link6':raise ValueError('Mount model requires flange-relative Dabai calibration')
    C=transform(he['position'],he['orientation'])
    size=np.asarray(cfg['body_box_size_m'],float);centre=np.asarray(cfg['body_center_optical_m'],float)
    anchor=np.asarray(cfg['bracket_flange_anchor_m'],float);thickness=float(cfg['bracket_thickness_m'])
    if (size.shape!=(3,) or centre.shape!=(3,) or anchor.shape!=(3,) or
        not np.isfinite(np.r_[size,centre,anchor,thickness]).all() or np.any(size<=0) or thickness<=0):
        raise ValueError('Invalid Dabai mount dimensions')
    model=ET.fromstring(robot_xml);semantic=ET.fromstring(semantic_xml)
    if model.find("link[@name='link6']") is None:raise ValueError('Piper flange link6 missing')
    if model.find("link[@name='dabai_mount_body']") is not None:raise ValueError('Duplicate Dabai mount model')
    link=ET.SubElement(model,'link',name='dabai_mount_body')
    def box(name,xyz,rpy,dimensions):
        for kind in ('visual','collision'):
            body=ET.SubElement(link,kind,name=name)
            ET.SubElement(body,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
            ET.SubElement(ET.SubElement(body,'geometry'),'box',size=' '.join(map(str,dimensions)))
            if kind=='visual':ET.SubElement(ET.SubElement(body,'material',name=name),'color',rgba='0.15 0.3 0.6 0.5')
    body_centre=(C@np.r_[centre,1])[:3]
    box('dabai_body',body_centre,Rotation.from_matrix(C[:3,:3]).as_euler('xyz'),size)
    direction=body_centre-anchor;length=float(np.linalg.norm(direction))
    if length<1e-6:raise ValueError('Dabai bracket anchor coincides with body centre')
    rotation,_=Rotation.align_vectors([direction/length],[[0,0,1]])
    box('dabai_bracket',(body_centre+anchor)*.5,rotation.as_euler('xyz'),[thickness,thickness,length])
    joint=ET.SubElement(model,'joint',name='dabai_mount_joint',type='fixed')
    ET.SubElement(joint,'parent',link='link6');ET.SubElement(joint,'child',link='dabai_mount_body')
    ET.SubElement(joint,'origin',xyz='0 0 0',rpy='0 0 0')
    # 相机与固定夹爪座属于同一刚性装配，近似包围体在安装处重叠。
    # 仅对模型中确实由固定关节连接到法兰的安装部件忽略内部接触。
    fixed_links = {'link6'}
    for _ in model.findall('joint'):
        for fixed in model.findall("joint[@type='fixed']"):
            if fixed.find('parent').get('link') in fixed_links:
                fixed_links.add(fixed.find('child').get('link'))
    for installed in ('link6', 'gripper_flange', 'gripper_base'):
        if installed in fixed_links:
            ET.SubElement(semantic,'disable_collisions',link1=installed,
                          link2='dabai_mount_body',reason='Adjacent')
    return ET.tostring(model,encoding='unicode'),ET.tostring(semantic,encoding='unicode')


def assert_visual_target_fresh(command, now=None):
    if not command.get('vision_target'):
        return
    try:
        age=(time.time() if now is None else now)-float(command.get('detection_stamp'))
    except (TypeError,ValueError):
        raise ValueError('视觉抓取目标缺少有效时间戳') from None
    if not -.05<=age<=1.:
        raise ValueError('视觉抓取目标已过期，请重新检测')
