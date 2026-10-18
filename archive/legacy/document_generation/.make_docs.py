from zipfile import ZipFile, ZIP_DEFLATED
from xml.sax.saxutils import escape
from pathlib import Path

OUT = Path('/home/lxf/agx_arm_ws/Piper动态抓取_开发与测试文档.docx')

def run(text, style=None):
    ppr = f'<w:pPr><w:pStyle w:val="{style}"/></w:pPr>' if style else ''
    return f'<w:p>{ppr}<w:r><w:t xml:space="preserve">{escape(text)}</w:t></w:r></w:p>'

def cell(text, head=False):
    shade = '<w:shd w:fill="D9EAF7"/>' if head else ''
    return f'<w:tc><w:tcPr><w:tcW w:w="2400" w:type="dxa"/>{shade}</w:tcPr>{run(text)}</w:tc>'

def table(rows):
    trs = ''.join('<w:tr>' + ''.join(cell(x, i == 0) for x in row) + '</w:tr>' for i, row in enumerate(rows))
    return '<w:tbl><w:tblPr><w:tblW w:w="0" w:type="auto"/><w:tblBorders>' + ''.join(f'<w:{b} w:val="single" w:sz="4" w:space="0" w:color="A6A6A6"/>' for b in ('top','left','bottom','right','insideH','insideV')) + '</w:tblBorders></w:tblPr>' + trs + '</w:tbl>'

body=[]
def title(t): body.append(run(t,'Title'))
def h(t): body.append(run(t,'Heading1'))
def h2(t): body.append(run(t,'Heading2'))
def para(t): body.append(run(t))
def bullet(t): body.append(run('• '+t))
def code(t): body.append(run(t,'Code'))

title('松灵 Piper 机械臂动态抓取系统\n开发与测试文档')
para('版本：V1.0　|　整理日期：2026-09-28　|　依据：工作空间源码、配置、README 与现有测试脚本')
para('本文按当前工作空间实现整理。硬件参数、标定文件和模型路径与设备部署绑定；执行前应以现场配置为准。本文件中的验收步骤是测试建议，除“仓库现有测试覆盖”标注外不代表已经在本次整理中运行或通过。')

h('1. 文档目的与范围')
para('说明 agx_arm_ws 中 Piper 动态抓取与分拣软件的模块边界、运行链路、关键算法、配置入口、部署操作和验证方法。范围包含 ROS 2 驱动/MoveIt、D455 与检测数据接口、GraspNet 可选服务、动态跟踪抓取节点、GUI/传送带接口、周期抓取服务、仿真与离线工具。')

h('2. 工作空间结构与职责')
body.append(table([
['路径/组件','职责'],
['src/agx_arm_ros/src/agx_arm_ctrl','Piper ROS 2 驱动、CAN 通信、状态反馈、控制接口与启动文件'],
['src/agx_arm_ros/src/agx_arm_moveit','机器人描述、MoveIt 规划配置、控制器配置及仿真启动'],
['src/agx_arm_ros/src/agx_arm_auto_sorting','排序周期服务、配置解析、数据采集/关节状态记录器'],
['auto_sorting_action.py','主要动作执行节点：静态/动态/GraspNet 抓取、规划、IK、状态与异常处理'],
['graspnet_service_node.py','订阅 D455 彩色/对齐深度与相机内参，推理并输出抓取候选'],
['sorting_gui_client.py','GUI、启动管线、检测显示、分拣控制及传送带继电器接口'],
['conveyor_calib.py / belt_calib_cli.py / velocity_predictor.py','传送带方向拟合、速度估计与延迟补偿预测'],
['stm32_conveyor.py / ch340-relay','串口传送带/继电器控制与状态读取'],
['conveyor.scene / table.scene','仿真规划场景几何'],
['tests/、offline_*、test_graspnet_hardware.sh','算法单测、IK 离线诊断及实机就绪性检查']]))

h('3. 总体架构与数据流')
para('系统采用 ROS 2 Humble 工作空间，真实机械臂经 CAN 驱动；MoveIt 负责运动规划。眼在手外 D455 固定于传送带上方，深度与 RGB 对齐。检测节点发布检测 JSON，动态动作节点将测量坐标转换/校验后预测未来抓取位置。传送带速度来自 STM32 状态，视觉轨迹速度可用于修正。GraspNet 服务路径通过独立请求/结果话题返回 6DoF 抓取候选。GUI 负责人工触发、目标料框选择及传送带操作。')
body.append(table([
['数据接口','用途'],
['/detection_info (std_msgs/String)','检测结果 JSON；含检测状态、类别、位置/时间戳等字段，供动作节点与二阶段复核使用'],
['/conveyor_status (std_msgs/String)','传送带速度/运行状态数据，VelocityPredictor 前馈输入'],
['/sorting_cmds (std_msgs/String)','GUI/客户端发布 sort、sort_verify、GraspNet 或传送带相关动作命令'],
['/graspnet/req → /graspnet/result (String)','独立 GraspNet 推理请求和候选抓取结果'],
['/control/joint_states、MoveIt action/services','关节命令、规划、IK 与笛卡尔路径接口'],
['/sorting_status、/sorting/cycle_result','动作状态及结构化周期结果'],
['/feedback/joint_states、/feedback/tcp_pose','Piper 实际状态/末端反馈；用于可视化和就绪检查'],
['/emergency_stop','GUI/节点间紧急停止信号接口']]))

h('4. 动态抓取实现说明')
h2('4.1 感知与坐标')
bullet('相机使用 640×480@30 配置并启用 align_depth；相机驱动 publish_tf:=false，由单独 eye-to-hand 静态 TF 提供 base_link 到 camera_color_optical_frame。启动 README 中给出的静态变换示例平移为 [0.649, -0.054, 0.596] m，使用前必须核对实际标定。')
bullet('检测节点融合目标类别、图像/深度信息和手眼外参，发布 /detection_info。目标偏移补偿可通过 x_offset_m、y_offset_m、z_offset_m、depth_offset_m 等参数调整。')
h2('4.2 速度估计与运动预测')
bullet('conveyor_calib.py 使用二维轨迹 PCA 主轴拟合传送带方向，并以 dx≥0 消除方向轴 180° 二义性；视觉速度通过时间-位置最小二乘斜率求取。')
bullet('VelocityPredictor 缓存最多 20 个检测位置样本；默认以 STM32 速度沿标定方向形成前馈速度，并按 0.85/0.15 权重融合视觉速度。预测式为 p_target = p_meas + v·(t_target−t_meas)。测量时刻应使用图像采集时间戳，以补偿 YOLO/深度/通信延迟。')
bullet('方向参数来自 conveyor_calib.json；动作逻辑应在速度无效、方向缺失或目标超出工作空间时采取拒绝/降级策略。标定角度轴的正负方向不能仅依赖 PCA，应结合实际皮带运行方向确认。')
h2('4.3 运动规划与抓取')
bullet('auto_sorting_action.py 内包含 Pinocchio IK 求解实现并使用多种姿态和多初始种子搜索；IK 后还需结合关节跳变、规划可行性和碰撞场景判断。规划可使用 MoveIt 服务/action，并发布关节控制轨迹。')
bullet('动态抓取逻辑先到悬停/影子位置，再按预测目标位置安排下降和夹持；抓取高度由物体表面高度、夹持点偏移和安全下限共同计算。源码示例常量包括 GRIPPER_PICK_Z_OFFSET=0.064 m、MIN_GRASP_Z=0.045 m、DESCENT_NOMINAL_S=0.6 s；P_HOVER 示例值明确标注为待标定，不能直接视为通用值。')
bullet('工作空间含传送带碰撞场景管理逻辑：接近/下降过程按需要调整 Allowed Collision Matrix，完成后恢复碰撞检测。必须检查失败路径是否恢复场景并安全退出。')
bullet('夹爪可结合目标尺寸动态计算开口与闭合位置，并通过关节反馈判断闭合误差；GraspNet 路径额外依据候选位姿/宽度执行抓取。')
h2('4.4 周期服务路径')
para('agx_arm_auto_sorting 的 sorting_cycle_server 提供 /sorting/prepare_once 与 /sorting/trigger_once，订阅 /sorting_cmds，状态发布到 /sorting/state、结果发布到 /sorting/cycle_result。sort_verify 命令会先到复核位等待新的检测，比较两次 XY 偏差（默认容差 0.03 m），随后使用复核坐标执行抓取。配置文件同时含工作空间、规划尺度、夹爪尺寸映射、抓放姿态和数据采集设置。该周期服务与根目录 auto_sorting_action 是不同执行实现，联调时应确认只启用预期动作入口，避免双节点抢控。')

h('5. 依赖与部署')
bullet('ROS 2 Humble；agx_arm_ros 依赖 pyAgxArm、python-can、numpy/scipy、ros2_control 与 MoveIt 等。实际依赖以各 package.xml、驱动 README 和设备环境为准。')
bullet('实机设备：Piper、CAN 适配器（can0，文档示例 1 Mbps）、D455、CH340/STM32 传送带控制器；根据需要连接夹爪。')
bullet('推理依赖：GraspNet 权重、PyTorch/CUDA 及数据集虚拟环境；检测节点在 orange_dataset 环境。具体绝对路径来自当前部署，不建议移植时照抄。')
code('cd /home/lxf/agx_arm_ws\ncolcon build\nsource install/setup.bash')
para('代码仓库附带 start_graspnet_pipeline.sh 前置检查及分终端启动指引。建议依次启动：T1 MoveIt+Piper 驱动；T2 D455；T2b 静态相机 TF；T3 检测；T4 GraspNet（可选）；T5 auto_sorting_action；T6 GUI。先确认节点/话题就绪，再使能执行抓取。实机与仿真不要并行控制同一机械臂。')

h('6. 关键参数与运行约束')
body.append(table([
['参数/配置','当前源码/配置示例','说明'],
['ROS/相机帧率','Humble；640×480@30','相机配置必须匹配对齐深度及检测节点'],
['传送带方向','conveyor_calib.json','PCA 轴方向还需用皮带正向确认'],
['速度融合权重','STM32 0.85，视觉 0.15','仅当视觉速度可用时融合'],
['周期服务工作空间','x:[0.02,0.62]、y:[-0.45,0.35]、z:[0.03,0.45] m','仅 sorting_cycle_server 配置'],
['规划速度/加速度比例','0.4 / 0.4','周期服务配置，不等同所有 MoveIt 入口'],
['夹爪尺寸映射','直径 0.04–0.08 m；开口 0.030–0.055 rad','周期服务配置示例'],
['动态抓取高度保护','MIN_GRASP_Z=0.045 m；夹持点偏移 0.064 m','auto_sorting_action 源码示例，现场需标定'],
['悬停位/下降提前量','P_HOVER 示例 z=0.32 m；标称下降 0.6 s','源码明确标注待标定，需按机械臂/皮带校准']]))

h('7. 操作流程与故障处理')
para('上电后检查急停、机械臂周边净空、夹爪安装和 CAN；激活 CAN 并使能机械臂；启动 MoveIt/驱动、相机与 TF、检测节点、可选 GraspNet、动作节点、GUI。先在低速/空载状态验证单点姿态与夹爪，再放置软质轻物测试静态抓取，最后进行移动目标测试。每次循环核对目标类别、预测位置、规划结果、夹爪反馈、放置位置和结果 JSON。')
body.append(table([
['现象','优先检查'],
['无检测结果/位置跳变','RGB/对齐深度、camera_info、时间戳、静态 TF、遮挡与深度滤波'],
['预测落点持续偏离','皮带方向正负、STM32 速度单位/状态、视觉速度和延迟、标定文件、相机外参'],
['IK/MoveIt 规划失败','目标是否在工作空间、当前关节状态、姿态候选、碰撞场景、控制器 action 状态'],
['抓空/夹不住','目标深度/表面高度、夹爪 TCP 偏移、物体直径、闭合角反馈及夹持点安全高度'],
['GraspNet 无结果','权重/设备、D455 对齐深度和内参、点云有效点数、请求/结果话题、服务日志'],
['传送带状态异常','CH340 端口权限、STM32 串口状态、继电器连接及 /conveyor_status']]))

test_start = len(body)
h('8. 测试文档')
para('测试目标：证明几何/预测模块、ROS 接口、MoveIt 规划、夹爪执行、感知推理和移动物体抓取可分别工作并最终闭环。每次记录软件版本、配置/标定文件版本、测试物体、皮带速度、运行次数、成功/失败原因与日志。')
h2('8.1 仓库已有测试与工具')
bullet('tests/test_conveyor_calib.py：方向拟合角度、180°二义性、点数不足异常、视觉速度估计。')
bullet('tests/test_velocity_predictor.py：速度方向投影、少样本处理、延迟预测、视觉速度恢复与融合。')
bullet('test_graspnet_hardware.sh：检查 RealSense USB、CAN、串口继电器、ROS topics、GraspNet 服务、MoveIt 节点，并打印手动调用提示；它只检查就绪，不执行自动抓取。')
bullet('offline_ik_multi_seed_test.py、offline_verify_jointspace_multiwaypoint.py：离线 IK 可达率/关节连续性分析，不驱动真机；结果依赖 URDF、标定文件、IK 环境与随机种子。')
bullet('tests/test_conveyor_calib.py 与 tests/test_velocity_predictor.py 是仓库明确可见的 pytest 单测；本次文档整理未运行测试。')
h2('8.2 建议测试用例')
body.append(table([
['ID / 级别','测试内容与步骤','通过标准/记录'],
['UT-01 单元','对传送带方向拟合使用已知角度合成轨迹；少于3点输入应抛 ValueError','角度误差满足现有断言 2°；方向单位长度；异常正确'],
['UT-02 单元','验证恒速预测、少样本、STM32/视觉融合','与解析期望一致；缺失视觉速度时回退前馈'],
['UT-03 单元','检查速度负值钳位、零时间跨度、方向归一化边界','输出有限且无 NaN；边界行为记录'],
['IT-01 接口','启动 ROS 后核对 camera image/depth/info、/detection_info、/conveyor_status','话题类型/发布者数量正确，时间戳递增'],
['IT-02 接口','检查 /graspnet/req/result 请求闭环，先用合法帧后用空/无效深度帧','有效请求返回候选或可解释失败；节点不崩溃，无请求则无假结果'],
['IT-03 规划','在仿真 MoveIt 中发送待机、预抓取、抓取、抬升和放置目标','动作成功返回；超工作空间目标被拒；规划场景恢复'],
['IT-04 夹爪','在无物条件下开合，检查轨迹 action 与 joint_states','夹爪目标到达，左右关节方向正确，反馈误差符合配置容差'],
['ST-01 静态抓取','固定目标、低速、轻质软质目标，重复 20 次','记录抓取/放置成功率、定位误差、碰撞/急停；目标建议≥18/20，失败原因可追溯'],
['ST-02 动态抓取','至少 3 个皮带速度档，各档重复 20 次，覆盖视野前/中/后位置','统计成功率、预测误差、动作耗时；项目阈值由团队确认并记录'],
['ST-03 变化场景','改变目标直径/类别、摆放姿态、光照，做遮挡与深度无效注入','无效感知不得盲目下探；异常能停止/跳过并返回错误原因'],
['ST-04 GraspNet','不同形状目标获取候选并执行若干次抓放','记录候选数、得分、位姿与夹持宽度；失败无节点崩溃'],
['REC-01 故障恢复','断开相机、GraspNet、夹爪反馈或规划服务，观察动作中止与复位','停止继续下探；场景碰撞矩阵恢复；系统回到可诊断状态'],
['REG-01 连续运行','自动分拣连续运行 30 分钟或 100 循环','无节点泄漏/失控/重复抢控；统计漏检、误检、失败、超时及恢复']]))
h2('8.3 实机安全测试步骤')
for s in [
'1) 清空工作区并确认急停有效；确保机械臂周边、传送带和料框无障碍。',
'2) 运行 bash test_graspnet_hardware.sh（节点依赖按脚本提示）；逐项确认设备与 ROS 接口。',
'3) 先关闭传送带，以低速度/低负载验证待机与空抓放轨迹；确认 TCP、夹爪方向与碰撞模型。',
'4) 固定目标执行静态抓取，校准相机到 base_link 变换和夹持高度；每次修改标定后留存参数快照。',
'5) 以最低可控皮带速度测试动态预测，随后逐档提高速度；出现抓取漂移、碰撞或延迟异常立即停止。',
'6) 注入遮挡/断流/超时等故障，确认节点报告失败且机械臂不继续执行危险下降。',
'7) 填写结果表：测试 ID、设备/软件版本、参数、速度、样本数、成功数、误差、异常日志、结论。']:
    para(s)

h('9. 验收记录模板')
body.append(table([
['字段','填写内容'],
['日期/人员',''],['代码版本 / ROS 发行版',''],['机械臂/夹爪/相机序列号',''],['手眼与传送带标定文件',''],['皮带方向/各档速度',''],['测试 ID 与目标类别/直径',''],['样本数 / 成功数 / 成功率',''],['预测位置误差 / 抓取耗时',''],['故障与日志路径',''],['结论 / 遗留问题','']]))

maintenance_start = len(body)
h('10. 已知限制与维护建议')
bullet('多个绝对路径指向当前开发机（/home/lxf/...），重部署前应改为 ROS package share、环境变量或 YAML 参数。')
bullet('眼在手外变换、悬停姿态、TCP 夹持点偏移、最小抓取高度均为现场安全关键参数；源码示例或历史标定不可未经核验复用。')
bullet('README 描述的根目录动作节点与包内周期服务并存，GUI 自动启动和 launch 管线可能存在多入口；部署时明确单一动作控制器并确认话题命名空间。')
bullet('动态预测使用常速度模型，对皮带加减速、物体打滑、检测时间戳错误及网络时延突变的适应能力有限；建议监测预测残差并设置最大外推时长。')
bullet('现有单测集中于标定与速度预测，尚未看到覆盖硬件动作状态机/抓取执行全路径的自动化测试；应通过仿真和实机测试补足并持续保存量化结果。')
para('主要参考：工作空间 README.md、start_graspnet_pipeline.sh、test_graspnet_hardware.sh、auto_sorting_action.py、graspnet_service_node.py、velocity_predictor.py、conveyor_calib.py、agx_arm_auto_sorting/sorting_cycle_server.py 与 tests/。')

dev_body = body[:test_start] + body[maintenance_start:]
test_body = [run('松灵 Piper 机械臂动态抓取系统\n测试文档', 'Title'),
             run('版本：V1.0　|　整理日期：2026-09-28　|　依据：工作空间源码、配置与现有测试脚本'),
             run('本文中的验收步骤为建议测试方案；除明确标注为仓库现有测试覆盖的部分外，不代表已在本次文档整理中执行或通过。')]
test_body += body[test_start:maintenance_start]

def write_docx(out_path, parts):
    content=''.join(parts)
    document=f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"><w:body>{content}<w:sectPr><w:pgSz w:w="11906" w:h="16838"/><w:pgMar w:top="1134" w:right="1134" w:bottom="1134" w:left="1134"/></w:sectPr></w:body></w:document>'''
    with ZipFile(out_path,'w',ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml',types)
        z.writestr('_rels/.rels',rels)
        z.writestr('word/document.xml',document)
        z.writestr('word/styles.xml',styles)
        z.writestr('word/_rels/document.xml.rels',docrels)
styles='''<?xml version="1.0" encoding="UTF-8"?><w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"><w:docDefaults><w:rPrDefault><w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="Microsoft YaHei"/><w:sz w:val="21"/></w:rPr></w:rPrDefault></w:docDefaults><w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/><w:pPr><w:spacing w:after="120" w:line="300" w:lineRule="auto"/></w:pPr></w:style><w:style w:type="paragraph" w:styleId="Title"><w:name w:val="Title"/><w:rPr><w:b/><w:color w:val="17365D"/><w:sz w:val="36"/><w:rFonts w:eastAsia="Microsoft YaHei"/></w:rPr><w:pPr><w:jc w:val="center"/><w:spacing w:after="300"/></w:pPr></w:style><w:style w:type="paragraph" w:styleId="Heading1"><w:name w:val="heading 1"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:rPr><w:b/><w:color w:val="1F4E79"/><w:sz w:val="29"/></w:rPr><w:pPr><w:keepNext/><w:spacing w:before="300" w:after="140"/></w:pPr></w:style><w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/><w:basedOn w:val="Normal"/><w:rPr><w:b/><w:color w:val="2F5597"/><w:sz w:val="24"/></w:rPr><w:pPr><w:keepNext/><w:spacing w:before="220" w:after="100"/></w:pPr></w:style><w:style w:type="paragraph" w:styleId="Code"><w:name w:val="Code"/><w:rPr><w:rFonts w:ascii="Consolas" w:eastAsia="Microsoft YaHei"/><w:sz w:val="18"/></w:rPr><w:pPr><w:ind w:left="300"/><w:shd w:fill="F2F2F2"/><w:spacing w:after="100"/></w:pPr></w:style></w:styles>'''
types='''<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/><Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/></Types>'''
rels='''<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/></Relationships>'''
docrels='''<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>'''
write_docx(Path('/home/lxf/agx_arm_ws/Piper动态抓取_开发文档.docx'), dev_body)
write_docx(Path('/home/lxf/agx_arm_ws/Piper动态抓取_测试文档.docx'), test_body)
print('Created development and test documents.')
