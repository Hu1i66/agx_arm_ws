#pragma once
#include <QObject>
#include <QVector>
#include <QMutex>
#include <QFuture>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
class MotionController:public QObject{
 Q_OBJECT
public:
 MotionController(rclcpp::Node::SharedPtr node,QObject *parent=nullptr);
 ~MotionController() override;
 void route(QVector<QVector<double>> points,double velocity,double acceleration,bool execute);
 void invalidate();void plan(QVector<double> joints,double velocity,double acceleration);void execute();void stop();
signals:void status(QString);
private:
 rclcpp::Node::SharedPtr node_;std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group_;
 moveit::planning_interface::MoveGroupInterface::Plan plan_;rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr publisher_;
 std::atomic<int> version_{0};std::atomic<int> plannedVersion_{-1};std::atomic<bool> busy_{false};QFuture<void> future_;
};
