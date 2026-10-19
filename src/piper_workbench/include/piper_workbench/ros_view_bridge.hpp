#pragma once
#include <QObject>
#include <QImage>
#include <QMap>
#include <QMutex>
#include <QJsonObject>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
class RosViewBridge:public QObject{
 Q_OBJECT
public:
 explicit RosViewBridge(const rclcpp::NodeOptions &,QObject *parent=nullptr);
 ~RosViewBridge() override;
 QImage image(QString camera,bool annotated=true);
 double imageAge(QString camera,bool annotated=true);
 rclcpp::Node::SharedPtr node;
signals:
 void detection(QString,QJsonObject);
 void joints(QVector<double>);
private:
 QMutex mutex_;QMap<QString,QImage> images_;QMap<QString,qint64> imageTimes_;
 std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
 rclcpp::executors::SingleThreadedExecutor executor_;std::thread thread_;
};
