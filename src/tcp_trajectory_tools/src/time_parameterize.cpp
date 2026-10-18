// Offline mathematics only: no ROS node, publisher, service or hardware access.
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
using json = nlohmann::json;
int main() {
  try {
    json input; std::cin >> input;
    const double duration=input.at("duration_s"), speed=input.at("speed");
    if (!std::isfinite(duration)||duration<=0||duration>18||!std::isfinite(speed)||speed<=0||speed>0.5)
      throw std::runtime_error("duration must be (0,18], speed must be (0,0.5]");
    std::list<Eigen::VectorXd> waypoints;
    for (const auto& row:input.at("positions")) {
      if (!row.is_array()||row.size()!=6) throw std::runtime_error("expected six joint positions");
      Eigen::VectorXd q(6);
      for(int i=0;i<6;++i) {q[i]=row[i].get<double>();if(!std::isfinite(q[i]))throw std::runtime_error("nonfinite joint");}
      if(waypoints.empty()||(q-waypoints.back()).cwiseAbs().maxCoeff()>1e-8)waypoints.push_back(q);
    }
    if(waypoints.size()<2) throw std::runtime_error("need distinct positions");
    Eigen::VectorXd velocity(6),acceleration(6);
    for(int i=0;i<6;++i) {
      velocity[i]=input.at("velocity").at(i).get<double>()*speed;
      acceleration[i]=input.at("acceleration").at(i).get<double>()*speed;
      if(!std::isfinite(velocity[i])||!std::isfinite(acceleration[i])||velocity[i]<=0||acceleration[i]<=0)
        throw std::runtime_error("invalid limits");
    }
    // A true reversal needs zero instantaneous velocity, but no dwell or new action goal.
    // Split reversals for TOTG; concatenate all segments into one timed trajectory.
    std::vector<Eigen::VectorXd> q(waypoints.begin(),waypoints.end());
    std::vector<std::unique_ptr<trajectory_processing::Path>> paths;
    std::vector<std::unique_ptr<trajectory_processing::Trajectory>> trajectories;
    std::vector<double> boundaries{0.};
    std::size_t begin=0;
    for(std::size_t end=1;end<q.size();++end) {
      bool split=end==q.size()-1;
      if(!split) {
        Eigen::VectorXd left=q[end]-q[end-1],right=q[end+1]-q[end];
        split=left.dot(right)/(left.norm()*right.norm()) < -0.5;
      }
      if(!split)continue;
      std::list<Eigen::VectorXd> segment(q.begin()+begin,q.begin()+end+1);
      paths.push_back(std::make_unique<trajectory_processing::Path>(segment,0.0001));
      auto trajectory=std::make_unique<trajectory_processing::Trajectory>(*paths.back(),velocity*0.95,acceleration*0.5,0.001);
      if(!trajectory->isValid())throw std::runtime_error("TOTG failed");
      const double d=trajectory->getDuration();
      if(!std::isfinite(d)||d<=0)throw std::runtime_error("invalid TOTG duration");
      boundaries.push_back(boundaries.back()+d);trajectories.push_back(std::move(trajectory));begin=end;
    }
    const double minimum=boundaries.back();
    if(minimum>duration*0.97)throw std::runtime_error("requested duration infeasible with 3% margin; minimum="+std::to_string(minimum));
    const double scale=duration/minimum;
    const int count=static_cast<int>(std::ceil(duration/0.02));
    std::vector<double> times;
    for(int k=0;k<=count;++k)times.push_back(duration*k/count);
    for(double t:boundaries)times.push_back(t*scale);
    std::sort(times.begin(),times.end());
    times.erase(std::unique(times.begin(),times.end(),[](double a,double b){return std::abs(a-b)<1e-9;}),times.end());
    json points=json::array();std::size_t index=0;
    for(double t:times) {
      const double native_t=t/scale;
      while(index+1<trajectories.size()&&native_t>boundaries[index+1]+1e-10)++index;
      const double local=std::clamp(native_t-boundaries[index],0.,trajectories[index]->getDuration());
      Eigen::VectorXd position=trajectories[index]->getPosition(local);
      Eigen::VectorXd v=trajectories[index]->getVelocity(local)/scale;
      const bool boundary=std::any_of(boundaries.begin(),boundaries.end(),[native_t](double b){return std::abs(native_t-b)<1e-9;});
      json point={{"time_s",t},{"positions",json::array()},{"velocities",json::array()},{"accelerations",json::array()}};
      for(int i=0;i<6;++i) {point["positions"].push_back(position[i]);point["velocities"].push_back(boundary?0.:v[i]);}
      points.push_back(point);
    }
    std::cout<<json({{"minimum_duration_s",minimum},{"duration_s",duration},{"path_tolerance_rad",0.0001},{"points",points}}).dump()<<std::endl;
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<std::endl; return 2;
  }
}
