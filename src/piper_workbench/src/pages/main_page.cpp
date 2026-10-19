#include "piper_workbench/main_window.hpp"
#include "piper_workbench/backend_client.hpp"
#include <algorithm>
using namespace Ui;
QWidget *MainWindow::makeMain(){
 auto *page=new QWidget;page->setObjectName("page");auto *layout=new QVBoxLayout(page);layout->setContentsMargins(20,20,20,16);layout->setSpacing(14);
 layout->addWidget(text("抓取工作台","pageTitle"));
 auto *flowCard=card("抓取流程 · 实时");flowCard->setMaximumHeight(150);auto *flowLayout=static_cast<QVBoxLayout *>(flowCard->layout());graspFlow_=new GraspFlow;flowLayout->addWidget(graspFlow_);layout->addWidget(flowCard);
 auto *split=new QSplitter;layout->addWidget(split,1);
 auto *cameras=new QWidget;auto *cl=new QVBoxLayout(cameras);cl->setContentsMargins(0,0,0,0);cl->setSpacing(12);
 for(QString cam:{"d455","dabai"}){
  auto *box=card(cam=="d455"?"D455 · 固定相机":"Dabai · 腕载相机");auto *bl=static_cast<QVBoxLayout *>(box->layout());
  auto *stats=text(cam=="d455"?"配置 640 × 480 · 30 帧/秒":"配置 640 × 480 · 15 帧/秒");stats->setObjectName("camera-stats-"+cam);bl->addWidget(stats);auto *view=new ImageView;view->picked=[this,cam](int id){selectTarget(cam,id);};images_[cam+":main"]=view;bl->addWidget(view,1);cl->addWidget(box,1);
 }
 split->addWidget(cameras);
 auto *model=card("机械臂实时视图");auto *ml=static_cast<QVBoxLayout *>(model->layout());ml->addWidget(text("基坐标系 base_link · 实际反馈"));mainHost_=new QWidget;mainHost_->setLayout(new QVBoxLayout);mainHost_->layout()->setContentsMargins(0,0,0,0);ml->addWidget(mainHost_,1);jointFeedback_=text("J1 —    J2 —    J3 —    J4 —    J5 —    J6 —\n等待真实关节反馈");ml->addWidget(jointFeedback_);split->addWidget(model);
 auto *operations=new QWidget;auto *ol=new QVBoxLayout(operations);ol->setContentsMargins(0,0,0,0);ol->setSpacing(12);
 auto *grasp=card("抓取操作");auto *gl=static_cast<QVBoxLayout *>(grasp->layout());gl->setSpacing(8);gl->setContentsMargins(14,12,14,12);gl->addWidget(text("抓取模式"));mode_=new QComboBox;mode_->addItems({"D455 动态抓取","D455 静态抓取","Dabai 静态抓取"});mode_->hide();gl->addWidget(mode_);
 auto *modes=new QHBoxLayout;auto *buttons=new QButtonGroup(page);buttons->setExclusive(true);
 QStringList labels={"D455 动态","D455 静态","Dabai 静态"};
 for(int i=0;i<3;++i){auto *b=button(labels[i]);b->setCheckable(true);b->setObjectName("modeButton");b->setMinimumHeight(44);buttons->addButton(b,i);modes->addWidget(b);if(i==0)b->setChecked(true);connect(b,&QPushButton::clicked,this,[this,i]{mode_->setCurrentIndex(i);});}
 connect(mode_,qOverload<int>(&QComboBox::currentIndexChanged),buttons,[buttons](int i){buttons->button(i)->setChecked(true);});gl->addLayout(modes);
 placementMode_=new QComboBox;placementMode_->addItems({"统一放置","按识别类别分配"});gl->addWidget(placementMode_);plan_=new QComboBox;gl->addWidget(plan_);
 auto *edit=button("编辑放置方案与类别映射",false,"debug");edit->setMaximumHeight(28);edit->setMinimumHeight(28);gl->addWidget(edit);connect(edit,&QPushButton::clicked,this,[this]{nav_->setCurrentRow(1);debugTabs_->setCurrentIndex(4);});
 grasp->setToolTip("放置方案在每个物体开始时固定；修改作用于下一物体");
 auto *start=button("开始动态抓取",true,"play"),*single=button("抓取所选目标",true,"grab"),*stop=button("本轮后停止",false,"stop");startTaskButton_=start;singleTaskButton_=single;stopTaskButton_=stop;single->hide();auto *runbar=new QHBoxLayout;runbar->addWidget(start,2);runbar->addWidget(single,2);runbar->addWidget(stop,1);gl->addLayout(runbar);
 connect(mode_,qOverload<int>(&QComboBox::currentIndexChanged),this,[this,start,single,operations](int i){if(auto *b=findChild<QPushButton *>("observationButton"))b->setText(i==0?"动态准备位":"观察位");start->setVisible(i!=2);single->setVisible(i!=0);selectedTrack_=-1;selectedSession_.clear();selectedTargetSnapshot_={};updateTargets();updateTaskButtons();Ui::fadeIn(operations);});
 connect(start,&QPushButton::clicked,this,[this]{runTask(true);});connect(single,&QPushButton::clicked,this,[this]{runTask(false);});
 connect(stop,&QPushButton::clicked,this,[this]{stopRequested_=true;updateTaskButtons();request(stopTaskButton_,"motion.pause_after_cycle",{},[this](QJsonValue){refresh();},"正在停止…",{},true);});
 connect(plan_,qOverload<int>(&QComboBox::activated),this,[this](int){savePlacements();});connect(placementMode_,qOverload<int>(&QComboBox::activated),this,[this](int){savePlacements();});ol->addWidget(grasp);
 auto *targetCard=card("检测目标 · 当前相机");auto *tl=static_cast<QVBoxLayout *>(targetCard->layout());selectedTargetLabel_=text("尚未选择目标","notice");tl->addWidget(selectedTargetLabel_);targets_=new QTableWidget(0,3);targets_->setHorizontalHeaderLabels({"目标","置信度","来源"});targets_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);targets_->verticalHeader()->hide();targets_->setSelectionBehavior(QAbstractItemView::SelectRows);targets_->setSelectionMode(QAbstractItemView::SingleSelection);targets_->setEditTriggers(QAbstractItemView::NoEditTriggers);targets_->setMinimumHeight(110);tl->addWidget(targets_,1);
 // Select on press, before the next 10 Hz detection update can rebuild the row.
 connect(targets_,&QTableWidget::cellPressed,this,[this](int row,int){auto *item=targets_->item(row,0);if(!item)return;selectTarget(camera(),item->data(Qt::UserRole).toInt());});ol->addWidget(targetCard,1);
 auto *belt=card("传送带与本轮状态");auto *bl=static_cast<QVBoxLayout *>(belt->layout());auto *br=new QHBoxLayout;
 for(auto pair:QList<QPair<QString,QString>>{{"启动","start"},{"停止","stop"},{"正转","forward"},{"反转","reverse"}}){auto *b=button(pair.first);br->addWidget(b);connect(b,&QPushButton::clicked,this,[this,pair,b]{request(b,"conveyor.send",{{"action",pair.second}},[this](QJsonValue){refresh();},"发送中…","已发送");});}bl->addLayout(br);taskState_=text("已暂停 · 等待你开始抓取","notice");bl->addWidget(taskState_);ol->addWidget(belt);operations->setMinimumHeight(760);split->addWidget(Ui::scroll(operations));
 split->setStretchFactor(0,27);split->setStretchFactor(1,44);split->setStretchFactor(2,29);split->setSizes({345,560,370});
 QTimer::singleShot(0,graspFlow_,[this]{Ui::fadeIn(graspFlow_);});QTimer::singleShot(80,cameras,[cameras]{Ui::fadeIn(cameras);});QTimer::singleShot(150,operations,[operations]{Ui::fadeIn(operations);});
 return page;
}
void MainWindow::updateTargets(){
 auto data=detections_[camera()];QString session=data["motion_session"].toString();
 if(selectedTrack_>=0&&selectedSession_!=session){selectedTrack_=-1;selectedSession_.clear();selectedTargetSnapshot_={};}
 QVector<QJsonObject> objects;for(auto value:data["objects"].toArray())objects<<value.toObject();
 std::sort(objects.begin(),objects.end(),[](const QJsonObject &a,const QJsonObject &b){return a["track_id"].toInt()<b["track_id"].toInt();});
 static const QMap<QString,QString> chinese={{"apple","苹果"},{"green_apple","青苹果"},{"orange","橙子"},{"banana","香蕉"},{"yellow_pear","黄梨"},{"green_pear","青梨"},{"strawberry","草莓"},{"peach","桃子"},{"lemon","柠檬"},{"mango","芒果"},{"star_fruit","杨桃"},{"pomegranate","石榴"}};
 QSignalBlocker blocker(targets_);targets_->setUpdatesEnabled(false);
 if(targets_->rowCount()!=objects.size())targets_->setRowCount(objects.size());
 bool selectedFound=false;
 for(int r=0;r<objects.size();++r){
  auto o=objects[r];QString name=o["object_name"].toString();QString label=chinese.value(name,name);int track=o["track_id"].toInt();
  for(int c=0;c<3;++c)if(!targets_->item(r,c))targets_->setItem(r,c,new QTableWidgetItem);
  if(targets_->item(r,0)->data(Qt::UserRole).toInt()!=track){targets_->item(r,0)->setData(Qt::UserRole,track);targets_->item(r,0)->setToolTip(name+" · "+session);}
  if(targets_->item(r,0)->text()!=label)targets_->item(r,0)->setText(label);
  QString confidence=QString::number(o["confidence"].toDouble(),'f',2);if(targets_->item(r,1)->text()!=confidence)targets_->item(r,1)->setText(confidence);
  QString source=camera()=="d455"?"D455":(o["class_source"].toString()=="d455_fusion"?"Dabai · D455 校验":"Dabai");if(targets_->item(r,2)->text()!=source)targets_->item(r,2)->setText(source);
  if(track==selectedTrack_){selectedFound=true;if(targets_->currentRow()!=r)targets_->selectRow(r);}
 }
 if(!selectedFound&&targets_->currentRow()>=0){targets_->clearSelection();targets_->setCurrentCell(-1,-1);}
 targets_->setUpdatesEnabled(true);
 if(selectedTargetLabel_){if(selectedTrack_<0){selectedTargetLabel_->setText("尚未选择目标");selectedTargetLabel_->setStyleSheet("");}else{QString name=selectedTargetSnapshot_["object_name"].toString();QString base=selectedTargetSnapshot_["base_position_m"].isObject()?QString(" · 基座 (%1, %2, %3)").arg(selectedTargetSnapshot_["base_position_m"].toObject()["x"].toDouble(),0,'f',3).arg(selectedTargetSnapshot_["base_position_m"].toObject()["y"].toDouble(),0,'f',3).arg(selectedTargetSnapshot_["base_position_m"].toObject()["z"].toDouble(),0,'f',3):QString();selectedTargetLabel_->setText(QString("已选择：%1  T%2%3%4").arg(name).arg(selectedTrack_).arg(base).arg(selectedFound?"":" · 等待检测刷新"));selectedTargetLabel_->setStyleSheet("color:#0E6B4B;background:#EAF8F1;");}}
 for(QString cam:{"d455","dabai"}){auto *v=images_[cam+":main"];v->detection=detections_[cam];v->selected=cam==camera()?selectedTrack_:-1;v->update();}
}
void MainWindow::runTask(bool continuous){
 if(!continuous&&selectedTrack_<0){showError("请先点击画面中的目标框或下方目标列表，再执行抓取");return;}
 graspStage_="target";if(graspFlow_) {graspFlow_->setActive(true);graspFlow_->setStage("target","目标已锁定");}
 auto *button=continuous?startTaskButton_:singleTaskButton_;stopRequested_=false;
 request(button,"motion.start",{{"mode",mode_->currentIndex()==0?"d455-dynamic":mode_->currentIndex()==1?"d455-static":"dabai-static"},{"continuous",continuous},{"track_id",selectedTrack_},{"session",selectedSession_},{"target_snapshot",selectedTargetSnapshot_}},[this](QJsonValue){refresh();},continuous?"正在启动抓取…":"正在派发目标…",{},true);
}
void MainWindow::selectTarget(QString cameraId,int trackId){
 if(camera()!=cameraId)mode_->setCurrentIndex(cameraId=="dabai"?2:1);
 auto data=detections_[cameraId];selectedTrack_=trackId;selectedSession_=data["motion_session"].toString();selectedTargetSnapshot_={};
 for(auto value:data["objects"].toArray()){auto obj=value.toObject();if(obj["track_id"].toInt()==trackId){selectedTargetSnapshot_=obj;break;}}
 updateTargets();for(QString cam:{"d455","dabai"})images_[cam+":main"]->update();
}
void MainWindow::refreshPlacements(){
 plan_->blockSignals(true);plan_->clear();teachPlan_->blockSignals(true);teachPlan_->clear();auto selection=placements_["selection"].toObject();for(auto item:placements_["plans"].toArray()){auto p=item.toObject();plan_->addItem(p["name"].toString(),p["id"].toString());teachPlan_->addItem(p["name"].toString(),p["id"].toString());}plan_->setCurrentIndex(plan_->findData(selection["default_plan_id"].toString()));placementMode_->setCurrentIndex(selection["mode"]=="class"?1:0);plan_->blockSignals(false);teachPlan_->blockSignals(false);}
void MainWindow::savePlacements(){auto selection=placements_["selection"].toObject();selection["default_plan_id"]=plan_->currentData().toString();selection["mode"]=placementMode_->currentIndex()==0?"single":"class";placements_["selection"]=selection;backend_->call("placements.save",{{"data",placements_}},[this](QJsonValue v){placements_=v.toObject();toast_->setText("放置选择已保存 · 下一物体生效");});}
