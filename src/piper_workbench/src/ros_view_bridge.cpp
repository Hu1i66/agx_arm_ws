#include "piper_workbench/ros_view_bridge.hpp"
#include <QJsonDocument>
#include <QDateTime>
#include <QMutexLocker>
RosViewBridge::RosViewBridge(const rclcpp::NodeOptions &options,QObject *parent):QObject(parent){
 auto opts=options;opts.use_global_arguments(false);opts.arguments({"--ros-args","-r","/joint_states:=/feedback/joint_states"});opts.automatically_declare_parameters_from_overrides(true);
 node=std::make_shared<rclcpp::Node>("piper_workbench_io",opts);
 for(auto camera:{QString("d455"),QString("dabai")}){
  std::string raw=camera=="d455"?"/camera/camera/color/image_raw":"/dabai/color/image_raw";
  std::string prefix=camera=="d455"?"":"/dabai";
  subs_.push_back(node->create_subscription<sensor_msgs::msg::Image>(raw,rclcpp::SensorDataQoS(),[this,camera](sensor_msgs::msg::Image::ConstSharedPtr m){
   QImage img;
   if(m->encoding=="rgb8"||m->encoding=="bgr8"){
    img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888).copy();if(m->encoding=="bgr8")img=img.rgbSwapped();
   }else if(m->encoding=="mono8")img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_Grayscale8).copy();
   else if(m->encoding=="rgba8"||m->encoding=="bgra8"){
    img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGBA8888).copy();if(m->encoding=="bgra8")img=img.rgbSwapped();
   }
   QMutexLocker lock(&mutex_);images_[camera]=img;imageTimes_[camera]=QDateTime::currentMSecsSinceEpoch();
  }));
  subs_.push_back(node->create_subscription<sensor_msgs::msg::CompressedImage>(prefix+"/yolo/annotated_image/compressed",rclcpp::SensorDataQoS(),[this,camera](sensor_msgs::msg::CompressedImage::ConstSharedPtr m){
   auto img=QImage::fromData(m->data.data(),int(m->data.size()));QMutexLocker lock(&mutex_);images_[camera+"-annotated"]=img;imageTimes_[camera+"-annotated"]=QDateTime::currentMSecsSinceEpoch();
  }));
  subs_.push_back(node->create_subscription<std_msgs::msg::String>(prefix+"/detection_info",10,[this,camera](std_msgs::msg::String::ConstSharedPtr m){emit detection(camera,QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object());}));
 }
 subs_.push_back(node->create_subscription<sensor_msgs::msg::Image>("/handeye/debug_image",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Image::ConstSharedPtr m){
  if(m->encoding!="bgr8"&&m->encoding!="rgb8")return;
  QImage img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888).copy();if(m->encoding=="bgr8")img=img.rgbSwapped();QMutexLocker lock(&mutex_);images_["calibration"]=img;imageTimes_["calibration"]=QDateTime::currentMSecsSinceEpoch();
 }));
 subs_.push_back(node->create_subscription<sensor_msgs::msg::JointState>("/feedback/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr m){
  QVector<double> result;for(int i=1;i<=6;++i){auto it=std::find(m->name.begin(),m->name.end(),"joint"+std::to_string(i));if(it==m->name.end())return;result.push_back(m->position[std::distance(m->name.begin(),it)]);}emit joints(result);
 }));
 executor_.add_node(node);thread_=std::thread([this]{executor_.spin();});
}
RosViewBridge::~RosViewBridge(){executor_.cancel();if(thread_.joinable())thread_.join();}
QImage RosViewBridge::image(QString camera,bool annotated){
 QMutexLocker lock(&mutex_);auto key=camera;auto now=QDateTime::currentMSecsSinceEpoch();
 if(annotated&&now-imageTimes_.value(camera+"-annotated",0)<1500)key+="-annotated";
 if(now-imageTimes_.value(key,0)>2000)return {};return images_.value(key);
}
