#include "piper_workbench/rviz_panel.hpp"
#include <QVBoxLayout>
#include <QColor>
#include <QVector3D>
#include <QTimer>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <QApplication>
#include <rviz_common/visualization_frame.hpp>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
namespace {
// The workbench owns planning via MotionController. The hidden stock panel must
// not create another MoveGroupInterface while its model loads (30s offline wait,
// with a constructor/shutdown race in the installed MoveIt build).
class WorkbenchPlanningDisplay:public moveit_rviz_plugin::MotionPlanningDisplay{
protected:
 void onRobotModelLoaded()override{
  auto *stockFrame=frame_;frame_=nullptr;
  moveit_rviz_plugin::MotionPlanningDisplay::onRobotModelLoaded();
  frame_=stockFrame;
 }
};
}
RvizPanel::RvizPanel(QWidget *parent) : QWidget(parent) {
  ros_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("piper_workbench_visualizer");
  for (const auto &p : node()->get_node_options().parameter_overrides())
    if (!node()->has_parameter(p.get_name())) node()->declare_parameter(p.get_name(), p.get_parameter_value());
  auto *layout = new QVBoxLayout(this); layout->setContentsMargins(0, 0, 0, 0);
  frame_ = new rviz_common::VisualizationFrame(ros_, this);
  frame_->setWindowFlags(Qt::Widget);
  layout->addWidget(frame_);
  frame_->setApp(qApp);
  frame_->setSplashPath("");
  frame_->initialize(ros_, QString::fromLocal8Bit(qgetenv("AGX_ARM_WS")) + "/config/workbench/workbench.rviz");
  manager_ = frame_->getManager();
  manager_->setFixedFrame("base_link");
  frame_->setHideButtonVisibility(false);
  frame_->menuBar()->hide(); frame_->statusBar()->hide();
  for (auto *bar : frame_->findChildren<QToolBar *>()) bar->hide();
  auto *grid = manager_->createDisplay("rviz_default_plugins/Grid", "地面网格", true);
  grid->subProp("Cell Size")->setValue(0.1);
  grid->subProp("Color")->setValue(QColor("#D4DAE3"));
  grid->subProp("Plane Cell Count")->setValue(20);
  planning_ = new WorkbenchPlanningDisplay;
  manager_->addDisplay(planning_,true);
  planning_->setName("机械臂与规划");planning_->setEnabled(true);
  if (planning_) {
    planning_->subProp("Planning Request")->subProp("Planning Group")->setValue("arm");
    planning_->subProp("Planning Request")->subProp("Goal State Color")->setValue(QColor("#66A3FF"));
    planning_->subProp("Planning Request")->subProp("Goal State Alpha")->setValue(0.4);
    planning_->subProp("Planning Request")->subProp("Interactive Marker Size")->setValue(.13);
    planning_->subProp("Planning Request")->subProp("Query Start State")->setValue(false);
    planning_->subProp("Planning Request")->subProp("Query Goal State")->setValue(false);
  }
  auto *view = manager_->getViewManager()->getCurrent();
  view->subProp("Distance")->setValue(1.3);
  view->subProp("Focal Point")->setValue(QVariant::fromValue(QVector3D(.15,0,.2)));
  view->subProp("Pitch")->setValue(0.55); view->subProp("Yaw")->setValue(0.7);
  auto *interact=manager_->getToolManager()->addTool("rviz_default_plugins/Interact");
  manager_->getToolManager()->setCurrentTool(interact);
  for (auto *dock : frame_->findChildren<rviz_common::PanelDockWidget *>()) dock->hide();
  manager_->startUpdate();
  auto *monitorTimer=new QTimer(this);
  connect(monitorTimer,&QTimer::timeout,this,[this,monitorTimer]{
    if(!planning_||!planning_->getRobotModel())return;
    auto monitor=planning_->getPlanningSceneMonitor();if(!monitor)return;
    monitor->startStateMonitor("/feedback/joint_states");monitorTimer->stop();
  });monitorTimer->start(300);

}
RvizPanel::~RvizPanel() = default;
void RvizPanel::setEditing(bool editing) {
  if (planning_) planning_->subProp("Planning Request")->subProp("Query Goal State")->setValue(editing);
}

QVector<double> RvizPanel::actualJoints() const{
 QVector<double> values;if(!planning_||!planning_->getRobotModel())return values;
 auto monitor=planning_->getPlanningSceneMonitor();if(!monitor||!monitor->getStateMonitor())return values;
 auto state=monitor->getStateMonitor()->getCurrentState();std::vector<double> joints;state->copyJointGroupPositions("arm",joints);for(double joint:joints)values<<joint;return values;
}
