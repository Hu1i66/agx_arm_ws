#include "piper_workbench/motion_controller.hpp"
#include <QtConcurrent>
#include <QElapsedTimer>
MotionController::MotionController(rclcpp::Node::SharedPtr node,QObject *parent):QObject(parent),node_(node){publisher_=node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",10);}
MotionController::~MotionController(){stop();future_.waitForFinished();}
void MotionController::invalidate(){++version_;plannedVersion_=-1;emit status("目标已修改，请重新规划");}
void MotionController::plan(QVector<double> joints,double velocity,double acceleration){
 if(busy_.exchange(true)){emit status("当前手动规划尚未结束");return;}
 int version=version_;plannedVersion_=-1;
 future_=QtConcurrent::run([this,joints,velocity,acceleration,version]{
  try{
   if(joints.size()!=6)throw std::runtime_error("目标关节应为六个值");
   if(!group_){auto group=std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,"arm",nullptr,rclcpp::Duration::from_seconds(2));std::atomic_store(&group_,group);}
   group_->setEndEffectorLink("tcp_link");group_->setStartStateToCurrentState();
   if(!group_->getCurrentState(2))throw std::runtime_error("等待真实关节反馈");
   group_->setMaxVelocityScalingFactor(velocity);group_->setMaxAccelerationScalingFactor(acceleration);group_->setPlanningTime(5.);
   group_->setJointValueTarget(std::vector<double>(joints.begin(),joints.end()));QElapsedTimer timer;timer.start();
   auto code=group_->plan(plan_);
   if(code==moveit::core::MoveItErrorCode::SUCCESS&&version==version_){
    plannedVersion_=version;moveit_msgs::msg::DisplayTrajectory display;display.trajectory_start=plan_.start_state_;display.trajectory.push_back(plan_.trajectory_);publisher_->publish(display);
    emit status(QString("规划成功 · 用时 %1 秒 · 轨迹已更新").arg(timer.elapsed()/1000.,0,'f',2));
   }else emit status(QString("规划未完成：返回码 %1；目标变化时请重新规划").arg(code.val));
  }catch(const std::exception &e){emit status("规划失败："+QString::fromUtf8(e.what()));}
  busy_=false;
 });
}
void MotionController::execute(){
 if(busy_.exchange(true)){emit status("手动操作正在执行");return;}
 if(!group_||plannedVersion_!=version_){busy_=false;emit status("请先对当前目标规划成功");return;}
 future_=QtConcurrent::run([this]{auto code=group_->execute(plan_);emit status(QString("执行结束 · 返回码 %1").arg(code.val));busy_=false;});
}
void MotionController::stop(){++version_;plannedVersion_=-1;auto group=std::atomic_load(&group_);if(group)group->stop();emit status("已发送停止手动执行请求");}

void MotionController::route(QVector<QVector<double>> points,double velocity,double acceleration,bool executeRoute){
 if(busy_.exchange(true)){emit status("当前手动操作尚未结束");return;}
 int version=++version_;plannedVersion_=-1;
 future_=QtConcurrent::run([this,points,velocity,acceleration,executeRoute,version]{
  try{
   if(!group_){auto group=std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,"arm",nullptr,rclcpp::Duration::from_seconds(2));std::atomic_store(&group_,group);}
   auto state=group_->getCurrentState(2);if(!state)throw std::runtime_error("等待实际关节反馈");
   group_->setMaxVelocityScalingFactor(velocity);group_->setMaxAccelerationScalingFactor(acceleration);
   moveit_msgs::msg::DisplayTrajectory display;std::vector<moveit::planning_interface::MoveGroupInterface::Plan> segments;
   for(auto target:points){
    if(target.size()!=6)throw std::runtime_error("示教点必须有六个关节值");
    group_->setStartState(*state);group_->setJointValueTarget(std::vector<double>(target.begin(),target.end()));
    moveit::planning_interface::MoveGroupInterface::Plan segment;auto result=group_->plan(segment);
    if(result!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("示教路线中的一段规划失败");
    if(segments.empty())display.trajectory_start=segment.start_state_;display.trajectory.push_back(segment.trajectory_);segments.push_back(segment);
    state->setJointGroupPositions("arm",std::vector<double>(target.begin(),target.end()));state->update();
   }
   publisher_->publish(display);emit status(QString("示教路线规划成功，共 %1 段").arg(segments.size()));
   if(executeRoute){for(const auto &segment:segments){if(version!=version_)throw std::runtime_error("示教执行已停止或目标已变化");auto result=group_->execute(segment);if(result!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("示教执行失败");}emit status("示教路线执行完成");}
  }catch(const std::exception &e){emit status("示教路线："+QString::fromUtf8(e.what()));}
  busy_=false;
 });
}
