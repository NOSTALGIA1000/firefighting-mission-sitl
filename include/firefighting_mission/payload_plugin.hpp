#ifndef FIREFIGHTING_MISSION_PAYLOAD_PLUGIN_HPP
#define FIREFIGHTING_MISSION_PAYLOAD_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <firefighting_mission/DropSupply.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <memory>
#include <mutex>
#include <string>

namespace firefighting_mission {

class PayloadPlugin : public gazebo::ModelPlugin {
 public:
  PayloadPlugin();
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  struct Slot {
    gazebo::physics::JointPtr payload_joint;
    gazebo::physics::JointPtr door_joint;
    gazebo::physics::LinkPtr payload_link;
    bool released = false;
  };

  bool Release(unsigned channel, std::string* reason);
  bool DropService(DropSupply::Request& request,
                   DropSupply::Response& response);
  void FireCallback(const std_msgs::BoolConstPtr& message);
  void RescueCallback(const std_msgs::BoolConstPtr& message);

  gazebo::physics::ModelPtr model_;
  Slot fire_;
  Slot rescue_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Subscriber fire_subscriber_;
  ros::Subscriber rescue_subscriber_;
  ros::ServiceServer drop_service_;
  std::mutex mutex_;
};

}  // namespace firefighting_mission

#endif
