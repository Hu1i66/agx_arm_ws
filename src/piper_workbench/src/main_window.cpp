#include "piper_workbench/main_window.hpp"
#include "piper_workbench/rviz_panel.hpp"
#include "piper_workbench/backend_client.hpp"
#include "piper_workbench/ros_view_bridge.hpp"
#include "piper_workbench/motion_controller.hpp"
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <cmath>
#include "piper_workbench/workbench_state.hpp"
using namespace Ui;
MainWindow::MainWindow(QWidget *parent):QMainWindow(parent){
 setWindowTitle("Piper 抓取工作台");resize(1600,900);setMinimumSize(1280,760);
 QFile css(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))+"/src/piper_workbench/resources/workbench.qss");if(css.open(QFile::ReadOnly))qApp->setStyleSheet(css.readAll()+QString("\nQDoubleSpinBox::up-arrow,QSpinBox::up-arrow{image:url(%1/src/piper_workbench/resources/icons/up.svg);width:14px;height:14px;}QDoubleSpinBox::down-arrow,QSpinBox::down-arrow{image:url(%1/src/piper_workbench/resources/icons/down.svg);width:14px;height:14px;}").arg(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))));
 backend_=new BackendClient(this);connect(backend_,&BackendClient::error,this,&MainWindow::showError);
 auto *root=new QWidget(this);auto *row=new QHBoxLayout(root);row->setContentsMargins(0,0,0,0);row->setSpacing(0);
 auto *side=new QFrame;side->setObjectName("sidebar");side->setFixedWidth(184);auto *sl=new QVBoxLayout(side);sl->setContentsMargins(0,0,0,0);
 auto *brand=text("Piper 工作台","brand");brand->setFixedHeight(72);brand->setAlignment(Qt::AlignCenter);sl->addWidget(brand);
 nav_=new QListWidget;nav_->setObjectName("navigation");nav_->setIconSize(QSize(22,22));
 QStringList names={"抓取","调试","连接","标定","图片采集","日志"},symbols={"grab","debug","connect","calibrate","capture","logs"};
 for(int i=0;i<6;++i)nav_->addItem(new QListWidgetItem(icon(symbols[i]),names[i]));sl->addWidget(nav_,1);sl->addWidget(text("  本地工作空间\n  Piper · ROS 2 Humble"));sl->addSpacing(18);row->addWidget(side);
 auto *content=new QWidget;auto *column=new QVBoxLayout(content);column->setContentsMargins(0,0,0,0);column->setSpacing(0);
 auto *header=new QFrame;header->setObjectName("header");header->setFixedHeight(72);auto *hl=new QHBoxLayout(header);hl->setContentsMargins(20,10,20,10);
 health_=text("");health_->hide();hl->addWidget(health_);
 for(QString name:{"机械臂","D455","Dabai","YOLO","MoveIt","动作","传送带"}){
  auto *badge=new QFrame;badge->setObjectName("healthBadge");auto *bl=new QHBoxLayout(badge);bl->setContentsMargins(9,5,9,5);bl->setSpacing(7);
  auto *glyph=new QLabel;glyph->setPixmap(icon(name=="D455"||name=="Dabai"?"capture":name=="机械臂"?"robot":"calibrate").pixmap(19,19));bl->addWidget(glyph);
  auto *label=new QLabel(name+"\n未启动");label->setObjectName("healthLabel");healthBadges_[name]=label;bl->addWidget(label);hl->addWidget(badge);
 }hl->addStretch();
 for(auto pair:QList<QPair<QString,QString>>{{"待机位","standby"},{"观察位","observe"},{"复位","reset"},{"急停","emergency_stop"}}){
  auto *b=button(pair.first,false,pair.second=="observe"?"eye":pair.second=="emergency_stop"?"stop":"calibrate");if(pair.second=="observe"){b->setText("动态准备位");b->setObjectName("observationButton");b->setToolTip("动态模式使用已有动态准备位；Dabai 使用可示教观察位；D455 静态使用待机参考位");}if(pair.second=="emergency_stop"){b->setObjectName("danger");b->setMinimumWidth(92);}
  hl->addWidget(b);connect(b,&QPushButton::clicked,this,[this,pair]{if(pair.second=="emergency_stop")motion_->stop();backend_->call("motion.send",{{"cmd",pair.second},{"camera_id",camera()},{"dynamic",mode_->currentIndex()==0},{"workbench",true}});});
 }
 column->addWidget(header);pages_=new QStackedWidget;column->addWidget(pages_,1);row->addWidget(content,1);setCentralWidget(root);
 rviz_=new RvizPanel(root);bridge_=new RosViewBridge(rviz_->node()->get_node_options(),this);motion_=new MotionController(bridge_->node,this);
 pages_->addWidget(makeMain());pages_->addWidget(makeDebug());pages_->addWidget(makeConnections());pages_->addWidget(makeCalibration());pages_->addWidget(makeCapture());pages_->addWidget(makeLogs());
 mainHost_->installEventFilter(this);motionHost_->installEventFilter(this);
 toast_=text("等待后台连接。打开窗口不会自动激活机械臂。","notice");toast_->setMinimumHeight(36);column->addWidget(toast_);
 auto *indicator=new QFrame(nav_->viewport());indicator->setFixedWidth(3);indicator->setStyleSheet("background:#1769FF;border-radius:1px;");
 connect(nav_,&QListWidget::currentRowChanged,this,[this,indicator](int index){
  auto rect=nav_->visualItemRect(nav_->item(index));auto *slide=new QPropertyAnimation(indicator,"geometry",indicator);slide->setDuration(190);slide->setEasingCurve(QEasingCurve::InOutCubic);slide->setStartValue(indicator->geometry());slide->setEndValue(QRect(Ui::px(4),rect.y()+Ui::px(9),Ui::px(3),Ui::px(30)));slide->start(QAbstractAnimation::DeleteWhenStopped);pages_->setCurrentIndex(index);moveRviz();
  auto *label=pages_->currentWidget()->findChild<QLabel *>("pageTitle");if(label){auto *effect=new QGraphicsOpacityEffect(label);label->setGraphicsEffect(effect);auto *a=new QPropertyAnimation(effect,"opacity",label);a->setDuration(180);a->setStartValue(.3);a->setEndValue(1.);a->start(QAbstractAnimation::DeleteWhenStopped);}
 });nav_->setCurrentRow(0);
 connect(debugTabs_,&QTabWidget::currentChanged,this,[this]{moveRviz();});
 connect(backend_,&BackendClient::online,this,[this](bool online){toast_->setText(online?"后台已连接 · 各组件可独立操作":"后台连接中 · 可继续查看配置与三维模型");if(online){refresh();refreshParameters();refreshFiles();}});
 connect(backend_,&BackendClient::event,this,&MainWindow::receive);
 connect(bridge_,&RosViewBridge::detection,this,[this](QString cam,QJsonObject detection){detections_[cam]=detection;updateTargets();},Qt::QueuedConnection);
 connect(bridge_,&RosViewBridge::joints,this,[this](QVector<double> values){lastActual_=values;QStringList fields;for(int i=0;i<values.size();++i)fields<<QString("J%1  %2°").arg(i+1).arg(values[i]*180/M_PI,0,'f',1);jointFeedback_->setText(fields.join("    "));},Qt::QueuedConnection);
 connect(motion_,&MotionController::status,this,[this](QString value){motionState_->setText(value);});
 if(rviz_->planning())connect(rviz_->planning(),&moveit_rviz_plugin::MotionPlanningDisplay::queryGoalStateChanged,this,[this]{if(!syncing_&&rviz_->planning()->getRobotModel()&&rviz_->planning()->getQueryGoalState()){
  std::vector<double> values;rviz_->planning()->getQueryGoalState()->copyJointGroupPositions("arm",values);QVector<double> target;for(double value:values)target<<value;
  if(WorkbenchState::targetChanged(observedGoal_,target)){observedGoal_=target;motion_->invalidate();}syncGoal();}});
 auto *frames=new QTimer(this);connect(frames,&QTimer::timeout,this,[this]{for(auto it=images_.begin();it!=images_.end();++it){if(!it.value()->isVisible())continue;QString cam=it.key().section(':',0,0);bool raw=it.key().contains(":raw");
 if(cam=="calibration"&&(bridge_->imageAge(cam,false)<0||bridge_->imageAge(cam,false)>2))cam=calibrationCamera_->currentIndex()==0?"d455":"dabai";
 it.value()->frame=bridge_->image(cam,!raw);it.value()->ageSeconds=bridge_->imageAge(cam,!raw);it.value()->update();}});frames->start(100);
 refreshTimer_=new QTimer(this);connect(refreshTimer_,&QTimer::timeout,this,[this]{if(backend_->connected()){refresh();if(pages_->currentIndex()==1&&debugTabs_->currentIndex()==3)refreshFiles();}});refreshTimer_->start(2000);
 if(backend_->connected()){refresh();refreshFiles();refreshParameters();toast_->setText("后台已连接 · 各组件可独立操作");}
 auto *logs=new QTimer(this);connect(logs,&QTimer::timeout,this,[this]{if(logsDirty_&&!logPause_->isChecked()&&pages_->currentIndex()==5){filterLogs();logsDirty_=false;}});logs->start(350);
 Ui::scaleNativeUi(this);
}
MainWindow::~MainWindow(){delete motion_;delete bridge_;}
void MainWindow::closeEvent(QCloseEvent *event){backend_->call("session.close");event->accept();}
void MainWindow::showPage(int index){nav_->setCurrentRow(index);}
QString MainWindow::camera()const{return mode_->currentIndex()==2?"dabai":"d455";}
void MainWindow::showError(QString message){toast_->setText("操作未完成："+message);toast_->setStyleSheet("color:#B4232D;background:#FFF0F0;padding:9px;");}
void MainWindow::refresh(){backend_->call("status.get",{},[this](QJsonValue v){applyStatus(v.toObject());});}
void MainWindow::applyStatus(QJsonObject state){
 status_=state;auto cams=state["cameras"].toObject();auto nodes=state["nodes"].toArray();QStringList names;for(auto n:nodes)names<<n.toString();
 auto cameraState=[&](QString c){return cams[c].toObject()["state"].toString()=="receiving"?"有画面":"等待数据";};
 health_->setText(QString("机械臂 %1   D455 %2   Dabai %3\nYOLO %4   MoveIt %5   动作节点 %6   传送带 %7").arg(state["feedback"].isObject()?"有反馈":"等待反馈",cameraState("d455"),cameraState("dabai"),names.contains("object_detector")?"运行中":"未运行",names.contains("move_group")?"运行中":"未运行",names.contains("piper_moveit_action_client")?"运行中":"未运行",state["conveyor"].toObject()["connected"].toBool()?"已连接":"未连接"));
 auto badge=[this](QString name,QString value){healthBadges_[name]->setText(name+"\n"+value);healthBadges_[name]->setStyleSheet(value=="连接异常"?"color:#C12A38":value=="有画面"||value=="已连接"?"color:#10865D":"color:#5B6D85");};
 badge("机械臂",state["reconnect"].toObject()["count"].toInt()>0?"连接异常":state["feedback"].isObject()?"有实际反馈":"等待反馈");badge("D455",cameraState("d455"));badge("Dabai",cameraState("dabai"));badge("YOLO",names.contains("object_detector")?"运行中":"未启动");badge("MoveIt",names.contains("move_group")?"运行中":"未启动");badge("动作",names.contains("piper_moveit_action_client")?"运行中":"未启动");badge("传送带",state["conveyor"].toObject()["connected"].toBool()?"已连接":"未连接");
 for(QString camera:{"d455","dabai"}){auto info=cams[camera].toObject();auto *stats=findChild<QLabel *>("camera-stats-"+camera);if(stats){QString link;double speed=info["usb_speed_mbps"].toDouble();if(speed>0)link=speed>=5000?" · USB 3":" · USB 2（建议直接连接 USB 3 口）";stats->setText(QString("640 × 480 · 输入 %1 帧/秒 · 检测 %2%3").arg(info["input_fps"].toDouble(),0,'f',1).arg(info["detection_age_s"].isDouble()?QString::number(info["detection_age_s"].toDouble(),'f',2)+" 秒前":"等待数据",link));}}
 for(auto c:state["components"].toArray()){
  auto o=c.toObject();QString key=o["component"].toString()+":"+o["camera_id"].toString();if(!componentStates_.contains(key))continue;
  QString value=o["state"].toString(),label=value=="running"?"进程运行中 · 等待实际数据核对":value=="command_completed"?"命令已执行 · 状态以反馈为准":value=="unknown"?"状态未知 · 可手动操作":"未启动";
  if(o["record"].isObject())label+="\nPID "+QString::number(o["record"].toObject()["pid"].toInt());componentStates_[key]->setText(label);
 }
 if(!state["config_errors"].toArray().isEmpty())showError(state["config_errors"].toArray().first().toString());
 if(placements_.isEmpty()){placements_=state["placements"].toObject();refreshPlacements();}
 auto task=state["task"].toObject();QString phase=task["phase"].toString();taskState_->setText(phase=="paused"?"已暂停 · 由你明确开始下一轮":phase=="grasping"?"当前物体抓放中":phase=="pause_pending"?"完成当前物体后暂停":"等待目标进入抓取区");
 if(state["reconnect"].toObject()["count"].toInt()>0)toast_->setText("机械臂连接异常，建议到「连接」执行碰撞后恢复；该警告不唯一代表碰撞");
 else if(state["feedback"].isObject()&&toast_->text().startsWith("机械臂连接异常")){toast_->setText("机械臂实际反馈已恢复 · 抓取保持暂停");toast_->setStyleSheet("");}
}
void MainWindow::moveRviz(){
 if(pages_->currentIndex()!=0&&!(pages_->currentIndex()==1&&debugTabs_->currentIndex()==0)){rviz_->hide();return;}
 auto *host=pages_->currentIndex()==0?mainHost_:motionHost_;
 // Keep the native Ogre/X11 window under one permanent parent; reparenting destroys its GL drawable.
 const QRect geometry(host->mapTo(centralWidget(),QPoint(0,0)),host->size());
 if(rviz_->geometry()!=geometry)rviz_->setGeometry(geometry);
 if(!rviz_->isVisible()){rviz_->show();rviz_->raise();}
 rviz_->setEditing(pages_->currentIndex()==1);
}
bool MainWindow::eventFilter(QObject *object,QEvent *event){
 if((object==mainHost_||object==motionHost_)&&(event->type()==QEvent::Resize||event->type()==QEvent::Move||event->type()==QEvent::Show))QTimer::singleShot(0,this,[this]{moveRviz();});
 return QMainWindow::eventFilter(object,event);
}

void MainWindow::receive(QString event,QJsonValue value){
 auto data=value.toObject();
 if(event=="vision.targets"){detections_[data["camera_id"].toString()]=data["detection"].toObject();updateTargets();}
 else if(event=="arm.control_reason")toast_->setText(data["detail"].toString()+" · "+data["original"].toString());
 else if(event=="arm.reconnect_required")toast_->setText(data["detail"].toString()+QString(" · 当前实例累计 %1 次").arg(data["count"].toInt()));
 else if(event=="arm.recovery.progress"){connectionState_->setText(QString("恢复步骤 %1  %2  %3").arg(data["step"].toInt()+1).arg(data["label"].toString(),data["state"].toString()=="completed"?"已完成":data["state"].toString()=="failed"?"失败":"执行中"));if(data.contains("error"))connectionState_->setText(connectionState_->text()+"："+data["error"].toString());connectionLog_->appendPlainText(QJsonDocument(data).toJson(QJsonDocument::Indented));}
 else if(event=="log.batch")for(auto line:data["lines"].toArray()){QString text=line.toString();connectionLog_->appendPlainText(text);if(data["source"].toString().startsWith("标定:"))calibrationLog_->appendPlainText(text);logRow({{"source",data["source"]},{"text",text},{"time",QDateTime::currentMSecsSinceEpoch()/1000.},{"level","INFO"}});}
 else if(event=="logs.rows")for(auto row:value.toArray())logRow(row.toObject());
 else if(event=="calibration.progress"||event=="calibration.result"){
  if(data["state"].toString()=="exited"){calibrationState_->setText(QString("工具已退出 · 返回码 %1 · 数据保留在 %2").arg(data["returncode"].toInt()).arg(data["directory"].toString()));return;}
  calibrationState_->setText(QString("采样 %1 组 · %2").arg(data["sample_count"].toInt()).arg(data["path"].toString()));
  auto raw=data["data"].toObject();resultText_->setPlainText(QJsonDocument(raw).toJson(QJsonDocument::Indented));
  auto records=raw["samples"].toArray();if(records.isEmpty())records=raw["frames"].toArray();if(!records.isEmpty()||data["path"].toString().endsWith("frames.json")){
   sampleTable_->setRowCount(records.size());for(int i=0;i<records.size();++i){auto r=records[i].toObject();auto *number=new QTableWidgetItem(QString::number(i+1));QString image=r["img"].toString();if(!image.isEmpty())number->setIcon(QIcon(QFileInfo(data["path"].toString()).dir().filePath(image)));sampleTable_->setItem(i,0,number);sampleTable_->setItem(i,1,new QTableWidgetItem(r.contains("stamp_s")?QString::number(r["stamp_s"].toDouble(),'f',3):"—"));sampleTable_->setItem(i,2,new QTableWidgetItem(r.contains("corners")?QString::number(r["corners"].toInt()):"—"));sampleTable_->setItem(i,3,new QTableWidgetItem(r.contains("reproj_rms_px")?QString::number(r["reproj_rms_px"].toDouble(),'f',3):"—"));number->setToolTip(QJsonDocument(r).toJson(QJsonDocument::Indented));sampleTable_->setRowHeight(i,46);}
  }
  if(event=="calibration.result")resultSummary_->setText("结果文件："+data["path"].toString()+"\n原质量："+raw["quality_status"].toString(raw["metadata"].toObject()["quality_status"].toString(raw["candidate"].toObject()["quality_status"].toString("见原始记录")))+"\n数值、误差与来源见下方原文；主动选用后才替换活动副本");
  if(event=="calibration.result")refreshFiles();
 }
 else if(event=="capture.progress"){
  if(data.contains("error")){showError(data["error"].toString());captureState_->setText("采集未完成："+data["error"].toString());return;}
  captureState_->setText(QString("已保存 %1 张\n%2\n%3").arg(data["count"].toInt()).arg(data["directory"].toString(),QJsonDocument(data["errors"].toObject()).toJson(QJsonDocument::Compact)));
  captureImages_->clear();for(auto row:data["images"].toArray()){auto image=row.toObject();auto *item=new QListWidgetItem(image["camera"].toString()+"  "+image["path"].toString(),captureImages_);item->setIcon(QIcon(data["directory"].toString()+"/"+image["path"].toString()));}
 }
 else if(event=="task.status"&&data.contains("error"))showError(data["error"].toString());
}
void MainWindow::showFile(QString path){QDesktopServices::openUrl(QUrl::fromLocalFile(path));}

QJsonObject MainWindow::inspect() const{
 QJsonArray actual,model,target;for(double v:lastActual_)actual.append(v);for(double v:rviz_->actualJoints())model.append(v);for(auto *v:joints_)target.append(v->value()*M_PI/180.);
 return {{"actual_feedback_rad",actual},{"rviz_state_rad",model},{"goal_rad",target},{"logical_width",width()},{"logical_height",height()},{"device_pixel_ratio",devicePixelRatioF()},{"interactive_end_effectors",rviz_->planning()&&rviz_->planning()->getRobotInteraction()?int(rviz_->planning()->getRobotInteraction()->getActiveEndEffectors().size()):0}};
}
