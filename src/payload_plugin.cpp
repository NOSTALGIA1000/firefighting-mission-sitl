#include "firefighting_mission/payload_plugin.hpp"

#include <gazebo/gazebo.hh>

namespace firefighting_mission {

PayloadPlugin::PayloadPlugin() = default;

void PayloadPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr) {
  model_ = model;
  fire_.payload_joint = model_->GetJoint("fire_payload_joint");
  fire_.door_joint = model_->GetJoint("fire_door_joint");
  fire_.payload_link = model_->GetLink("fire_payload_link");
  rescue_.payload_joint = model_->GetJoint("rescue_payload_joint");
  rescue_.door_joint = model_->GetJoint("rescue_door_joint");
  rescue_.payload_link = model_->GetLink("rescue_payload_link");
  if (!fire_.payload_joint || !fire_.door_joint || !fire_.payload_link ||
      !rescue_.payload_joint || !rescue_.door_joint || !rescue_.payload_link) {
    gzerr << "PayloadPlugin: required payload links or joints are missing\n";
    return;
  }
  if (!ros::isInitialized()) {
    gzerr << "PayloadPlugin: ROS is not initialized; load gazebo_ros_api_plugin\n";
    return;
  }
  node_.reset(new ros::NodeHandle("fire_iris"));
  fire_subscriber_ = node_->subscribe("drop_fire", 1,
                                      &PayloadPlugin::FireCallback, this);
  rescue_subscriber_ = node_->subscribe("drop_rescue", 1,
                                        &PayloadPlugin::RescueCallback, this);
}

void PayloadPlugin::Release(unsigned channel) {
  std::lock_guard<std::mutex> lock(mutex_);
  Slot& slot = channel == 1 ? fire_ : rescue_;
  if (slot.released || !slot.payload_joint) return;
  slot.door_joint->SetPosition(0, channel == 1 ? 1.15 : -1.15);
  slot.payload_joint->Detach();
  slot.payload_link->SetGravityMode(true);
  slot.released = true;
  gzmsg << "PayloadPlugin released channel " << channel << "\n";
}

void PayloadPlugin::FireCallback(const std_msgs::BoolConstPtr& message) {
  if (message->data) Release(1);
}

void PayloadPlugin::RescueCallback(const std_msgs::BoolConstPtr& message) {
  if (message->data) Release(2);
}

GZ_REGISTER_MODEL_PLUGIN(PayloadPlugin)

}  // namespace firefighting_mission
