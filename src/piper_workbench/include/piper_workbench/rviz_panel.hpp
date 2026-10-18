#pragma once
#include <QWidget>
#include <QVector>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
namespace rviz_common { class VisualizationFrame; class VisualizationManager; }
namespace moveit_rviz_plugin { class MotionPlanningDisplay; }
class RvizPanel : public QWidget {
  Q_OBJECT
public:
  explicit RvizPanel(QWidget *parent = nullptr);
  ~RvizPanel() override;
  void setEditing(bool editing);
  QVector<double> actualJoints() const;
  rclcpp::Node::SharedPtr node() const { return ros_->get_raw_node(); }
  moveit_rviz_plugin::MotionPlanningDisplay *planning() const { return planning_; }
signals:
  void statusChanged(QString);
private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> ros_;
  rviz_common::VisualizationFrame *frame_;
  rviz_common::VisualizationManager *manager_;
  moveit_rviz_plugin::MotionPlanningDisplay *planning_ = nullptr;
};
