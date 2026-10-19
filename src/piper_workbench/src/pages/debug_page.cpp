#include "piper_workbench/main_window.hpp"
#include "piper_workbench/backend_client.hpp"
#include "piper_workbench/motion_controller.hpp"
#include "piper_workbench/rviz_panel.hpp"
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/robot_state.h>
#include <cmath>
using namespace Ui;
QWidget *MainWindow::makeDebug(){
 auto *page=new QWidget;page->setObjectName("page");auto *layout=new QVBoxLayout(page);layout->setContentsMargins(20,20,20,16);layout->addWidget(text("调试与参数","pageTitle"));debugTabs_=new QTabWidget;layout->addWidget(debugTabs_,1);
 debugTabs_->addTab(makeMotion(),"运动调试");debugTabs_->addTab(makeParameters(),"抓取参数");debugTabs_->addTab(makeCameraSettings(),"相机与检测");debugTabs_->addTab(makeFiles(),"标定文件");debugTabs_->addTab(makeTeach(),"示教与放置点");debugTabs_->addTab(makeAdvanced(),"高级");return page;
}
QWidget *MainWindow::makeMotion(){
 auto *page=new QWidget;auto *row=new QHBoxLayout(page);row->setContentsMargins(0,12,0,0);auto *model=card("运动规划 · 实际模型与目标影子");auto *ml=static_cast<QVBoxLayout *>(model->layout());
 motionHost_=new QWidget;motionHost_->setLayout(new QVBoxLayout);motionHost_->layout()->setContentsMargins(0,0,0,0);ml->addWidget(motionHost_,1);
 auto *buttons=new QHBoxLayout;
 for(QString label:{"读取当前位置","规划","执行","停止","清除预览"}){auto *b=button(label,label=="规划"||label=="执行",label=="执行"?"play":label=="停止"?"stop":"calibrate");buttons->addWidget(b);
  connect(b,&QPushButton::clicked,this,[this,label,b]{
   if(label=="读取当前位置"){request(b,"poses.capture",{},[this](QJsonValue v){QVector<double> j;for(auto x:v.toObject()["joints_rad"].toArray())j<<x.toDouble();setGoal(j);},"正在读取…","已读取");}
   if(label=="规划"){auto *p=rviz_->planning();if(!p||!p->getRobotModel()||!p->getQueryGoalState()){showError("机器人模型尚未加载");return;}std::vector<double> exact;p->getQueryGoalState()->copyJointGroupPositions("arm",exact);QVector<double> values;for(double q:exact)values<<q;motion_->plan(values,velocity_->value()/100.,acceleration_->value()/100.);}
   if(label=="执行")motion_->execute();if(label=="停止")motion_->stop();if(label=="清除预览"){motion_->invalidate();if(rviz_->planning())rviz_->planning()->dropVisualizedTrajectory();}
  });
 }ml->addLayout(buttons);motionState_=text("拖动只改变目标，点击执行才运动","notice");ml->addWidget(motionState_);row->addWidget(model,3);
 auto *side=new QWidget;auto *sl=new QVBoxLayout(side);sl->setContentsMargins(0,0,0,0);auto *jointCard=card("目标关节角");auto *jl=static_cast<QVBoxLayout *>(jointCard->layout());
 for(int i=0;i<6;++i){auto *line=new QHBoxLayout;line->addWidget(text("J"+QString::number(i+1)));auto *slider=new QSlider(Qt::Horizontal);slider->setRange(-1800,1800);line->addWidget(slider,1);auto *number=new QDoubleSpinBox;number->setRange(-180,180);number->setDecimals(2);number->setSuffix(" °");number->setMinimumWidth(112);joints_<<number;line->addWidget(number);jl->addLayout(line);
  connect(slider,&QSlider::valueChanged,number,[number](int v){number->setValue(v/10.);});connect(number,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this,slider](double v){QSignalBlocker block(slider);slider->setValue(qRound(v*10));if(!syncing_){QVector<double> j;for(auto *n:joints_)j<<n->value()*M_PI/180.;if(j.size()==6)setGoal(j);}});
 }jointCard->setMinimumHeight(330);sl->addWidget(jointCard);
 auto *tcpCard=card("TCP 目标位姿 · base_link");auto *tl=static_cast<QVBoxLayout *>(tcpCard->layout());auto *grid=new QGridLayout;
 QStringList labels={"X","Y","Z","滚转","俯仰","偏航"};
 for(int i=0;i<6;++i){auto *n=new QDoubleSpinBox;n->setRange(i<3?-1000:-180,i<3?1000:180);n->setDecimals(2);n->setSuffix(i<3?" mm":" °");tcp_<<n;grid->addWidget(text(labels[i]),(i/3)*2,i%3);grid->addWidget(n,(i/3)*2+1,i%3);}
 tl->addLayout(grid);auto *ik=button("更新末端目标",false,"calibrate");tl->addWidget(ik);connect(ik,&QPushButton::clicked,this,[this]{
  auto *p=rviz_->planning();if(!p||!p->getRobotModel()||!p->getQueryGoalState()){showError("机器人模型尚未加载");return;}auto state=*p->getQueryGoalState();Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()=Eigen::Vector3d(tcp_[0]->value()/1000.,tcp_[1]->value()/1000.,tcp_[2]->value()/1000.);
  pose.linear()=(Eigen::AngleAxisd(tcp_[5]->value()*M_PI/180.,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(tcp_[4]->value()*M_PI/180.,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(tcp_[3]->value()*M_PI/180.,Eigen::Vector3d::UnitX())).toRotationMatrix();
  if(!state.setFromIK(state.getJointModelGroup("arm"),pose,"tcp_link",.2)){showError("此末端目标无可用逆解，请调整位置或姿态");return;}p->setQueryGoalState(state);
 });tcpCard->setMinimumHeight(270);sl->addWidget(tcpCard);
 auto *scaling=card("规划参数 · 仅用于手动规划");auto *form=new QFormLayout;velocity_=new QDoubleSpinBox;acceleration_=new QDoubleSpinBox;for(auto *n:{velocity_,acceleration_}){n->setRange(1,100);n->setValue(20);n->setSuffix(" %");}form->addRow("速度比例",velocity_);form->addRow("加速度比例",acceleration_);static_cast<QVBoxLayout *>(scaling->layout())->addLayout(form);auto *saveManual=button("保存手动规划比例",false,"save");static_cast<QVBoxLayout *>(scaling->layout())->addWidget(saveManual);connect(saveManual,&QPushButton::clicked,this,[this,saveManual]{request(saveManual,"parameters.apply",{{"group","manual"},{"values",QJsonObject{{"velocity",velocity_->value()/100.},{"acceleration",acceleration_->value()/100.}}}},[this](QJsonValue){toast_->setText("手动规划比例已保存 · 不改变抓取速度");},"正在应用…","已应用");});scaling->setMinimumHeight(200);sl->addWidget(scaling);sl->addStretch();side->setMinimumHeight(800);row->addWidget(Ui::scroll(side),2);return page;
}
void MainWindow::setGoal(QVector<double> values){
 if(values.size()!=6)return;syncing_=true;
 for(int i=0;i<6;++i)joints_[i]->setValue(values[i]*180/M_PI);
 auto *p=rviz_->planning();if(p&&p->getRobotModel()&&p->getQueryGoalState()){auto state=*p->getQueryGoalState();state.setJointGroupPositions("arm",std::vector<double>(values.begin(),values.end()));state.update();p->setQueryGoalState(state);}syncing_=false;observedGoal_=values;motion_->invalidate();syncGoal();
}
void MainWindow::syncGoal(){
 auto *p=rviz_->planning();if(!p||!p->getRobotModel()||!p->getQueryGoalState())return;syncing_=true;auto state=p->getQueryGoalState();std::vector<double> values;state->copyJointGroupPositions("arm",values);
 for(size_t i=0;i<std::min(values.size(),size_t(6));++i)joints_[i]->setValue(values[i]*180/M_PI);
 auto pose=state->getGlobalLinkTransform("tcp_link");auto angles=pose.rotation().eulerAngles(2,1,0);for(int i=0;i<3;++i){tcp_[i]->setValue(pose.translation()[i]*1000.);tcp_[i+3]->setValue(angles[2-i]*180/M_PI);}syncing_=false;
}
QWidget *MainWindow::makeParameters(){
 auto *page=new QWidget;auto *row=new QHBoxLayout(page);row->setContentsMargins(0,12,0,0);
 for(QString group:{"static","dynamic"}){auto *box=card(group=="static"?"静态抓取参数":"动态抓取 · 原配置");auto *layout=static_cast<QVBoxLayout *>(box->layout());auto *fields=new QWidget;fields->setObjectName("fields-"+group);fields->setLayout(new QFormLayout);layout->addWidget(Ui::scroll(fields),1);auto *apply=button("保存并应用于下一物体",true,"save");layout->addWidget(apply);connect(apply,&QPushButton::clicked,this,[this,group,apply]{QJsonObject values;for(auto it=parameterFields_[group].begin();it!=parameterFields_[group].end();++it)values[it.key()]=parameterGroups_[group].toObject()[it.key()].isDouble()?QJsonValue(it.value()->value()):QJsonValue(it.value()->value());request(apply,"parameters.apply",{{"group",group},{"values",values}},[this](QJsonValue v){toast_->setText("参数已保存 · 下一物体使用新值");refreshParameters();},"正在应用…","已应用");});row->addWidget(box,group=="static"?1:2);}
 return page;
}
void MainWindow::refreshParameters(){backend_->call("parameters.get",{},[this](QJsonValue value){
 parameterGroups_=value.toObject();auto vision=parameterGroups_["vision"].toObject();for(auto it=parameterFields_["vision"].begin();it!=parameterFields_["vision"].end();++it)if(vision.contains(it.key()))it.value()->setValue(vision[it.key()].toDouble());auto manual=parameterGroups_["manual"].toObject();velocity_->setValue(manual["velocity"].toDouble(.2)*100);acceleration_->setValue(manual["acceleration"].toDouble(.2)*100);
 const QMap<QString,QString> labels={{"joint_velocity_scaling","关节速度比例 (0–1)"},{"joint_acceleration_scaling","关节加速度比例 (0–1)"},{"approach_height_m","工具轴预抓距离 (m)"},{"soft_lift_m","小幅抬起 (m)"},{"gripper_force_n","夹持力 (N)"},{"gripper_settle_s","夹爪稳定时间 (s)"},{"pregrasp_m","预抓取距离 (m)"},{"extra_depth_m","额外下降 (m)"},{"align_s","对齐时间 (s)"},{"approach_s","接近时间 (s)"},{"follow_s","随动时间 (s)"},{"close_s","闭合时间 (s)"},{"lift_s","抬起时间 (s)"},{"lift_m","抬起高度 (m)"},{"sample_s","轨迹采样间隔 (s)"},{"send_margin_s","发送预留 (s)"},{"observation_max_age_s","视觉最大年龄 (s)"},{"feedback_max_age_s","反馈最大年龄 (s)"},{"belt_max_age_s","传送带最大年龄 (s)"},{"velocity_scaling","速度比例 (0–1)"},{"acceleration_scaling","加速度比例 (0–1)"},{"max_plan_attempts","规划尝试次数"},{"max_stage_rounds","准备重试次数"},{"intercept_s_pref_m","偏好拦截位置 (m)"},{"intercept_window_m","拦截窗口 (m)"},{"intercept_s_max_m","最远拦截位置 (m)"},{"max_trajectory_s","最长轨迹 (s)"},{"stage_move_s","准备运动时间 (s)"},{"max_wait_s","最长等待 (s)"}};
 for(QString group:{"static","dynamic"}){auto *fields=findChild<QWidget *>("fields-"+group);auto *form=static_cast<QFormLayout *>(fields->layout());auto values=parameterGroups_[group].toObject();for(auto it=values.begin();it!=values.end();++it){auto *n=parameterFields_[group].value(it.key());if(!n){n=new QDoubleSpinBox;n->setDecimals(it.key().contains("attempts")||it.key().contains("rounds")?0:4);n->setRange(-1000,1000);n->setSingleStep(it.key().contains("attempts")||it.key().contains("rounds")?1:.01);n->setToolTip(it.key());form->addRow(labels.value(it.key(),it.key()),n);parameterFields_[group][it.key()]=n;}n->setValue(it.value().toDouble());}}
 });}
QWidget *MainWindow::makeCameraSettings(){
 auto *page=new QWidget;auto *row=new QHBoxLayout(page);row->setContentsMargins(0,12,0,0);
 for(QString camera:{"d455","dabai"}){auto *box=card(camera=="d455"?"D455 相机与检测":"Dabai 相机与检测");auto *layout=static_cast<QVBoxLayout *>(box->layout());
 auto *previewRow=new QHBoxLayout;auto *preview=new ImageView;preview->setFixedSize(300,180);preview->waiting="等待相机原图";images_[camera+":exposure:raw"]=preview;previewRow->addWidget(preview);previewRow->addStretch();layout->addLayout(previewRow);
 auto *form=new QFormLayout;auto *autoExposure=new QCheckBox("自动曝光");autoExposure->setChecked(true);auto *exposure=new QSpinBox;exposure->setRange(1,1000000);exposure->setValue(5000);form->addRow(autoExposure);form->addRow("手动曝光值",exposure);layout->addLayout(form);auto *applyExposure=button("应用曝光设置",true);layout->addWidget(applyExposure);
  if(camera=="d455"){auto *beltCalibrate=button("标定 D455 传送带范围",false,"calibrate");auto *beltState=text("固定相机：保存绿色台面多边形，只接受区域内的检测目标","notice");layout->addWidget(beltCalibrate);layout->addWidget(beltState);connect(beltCalibrate,&QPushButton::clicked,this,[this,beltCalibrate,beltState]{request(beltCalibrate,"belt.calibrate",{},[this,beltState](QJsonValue v){beltState->setText(QString("已保存台面范围 · %1 个顶点 · 视觉节点将自动加载").arg(v.toObject()["polygon"].toArray().size()));toast_->setText("D455 传送带范围已保存");},"正在标定…","标定成功");});}
  connect(applyExposure,&QPushButton::clicked,this,[this,camera,autoExposure,exposure,applyExposure]{request(applyExposure,"parameters.apply",{{"group","camera:"+camera},{"values",QJsonObject{{"auto",autoExposure->isChecked()},{"value",exposure->value()}}}},[this](QJsonValue v){toast_->setText("相机驱动已确认曝光参数");},"正在应用…","已应用");});
  auto *detectionForm=new QFormLayout;QMap<QString,QDoubleSpinBox *> values;for(auto pair:QList<QPair<QString,QString>>{{"yolo_conf","识别置信度 (0–1)"},{"yolo_iou","重叠抑制阈值 (0–1)"},{"detect_period_s","检测周期 (s)"}}){auto *n=new QDoubleSpinBox;n->setDecimals(3);n->setRange(.001,pair.first=="detect_period_s"?10:1);n->setValue(pair.first=="yolo_conf"?.25:pair.first=="yolo_iou"?.7:.1);values[pair.first]=n;parameterFields_["vision"][camera+"."+pair.first]=n;detectionForm->addRow(pair.second,n);}layout->addLayout(detectionForm);auto *apply=button("应用检测参数",true);layout->addWidget(apply);connect(apply,&QPushButton::clicked,this,[this,camera,values,apply]{QJsonObject data;for(auto it=values.begin();it!=values.end();++it)data[camera+"."+it.key()]=it.value()->value();request(apply,"parameters.apply",{{"group","vision"},{"values",data}},[this](QJsonValue){toast_->setText("检测节点接受更新 · 下一帧边界提交");},"正在应用…","已应用");});layout->addWidget(text(camera=="d455"?"D455 使用一次性固定台面范围。":"Dabai 每帧按绿色阈值提取台面范围，机械臂移动后自动更新。"));layout->addStretch();row->addWidget(box);}
 return Ui::scroll(page);
}
