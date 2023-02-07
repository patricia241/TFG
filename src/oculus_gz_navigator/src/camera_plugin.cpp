#include <chrono>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

namespace oculus_gz_navigator
{
  class CameraPositionPlugin : public gazebo::ModelPlugin
  {

    public: 
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            std::cout << "-----------------------------------------" << std::endl;
            std::cout << "CAMERA POSITION PLUGIN" << std::endl;
            std::cout << "-----------------------------------------" << std::endl;
            std::cout << std::endl;

            link_ = _model->GetLink("link");

            // ROS node
            node_ = gazebo_ros::Node::Get(_sdf);

            // ROS subscription
            pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>("camera_position", 10,
                                                                              std::bind(&CameraPositionPlugin::camera_position_callback,
                                                                                        this, std::placeholders::_1));
        }
    private:
        gazebo::physics::LinkPtr link_;

        // GazeboROS node 
        gazebo_ros::Node::SharedPtr node_;

        // ROS subscriptor
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

        // If in msg Position I get (0, 0, 1) increment z position 1m. Rotation set directly with msg info. 
        void camera_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
          ignition::math::Vector3d accel(0, 0, 0); 
          ignition::math::Pose3d pose = link_->WorldPose();

          pose.Pos().X() = pose.Pos().X() + msg->position.x;
          pose.Pos().Y() = pose.Pos().Y() + msg->position.y;
          pose.Pos().Z() = pose.Pos().Z() + msg->position.z;
          link_->SetWorldPose(pose);

          link_->SetLinearVel(accel);
        }
  };

  GZ_REGISTER_MODEL_PLUGIN(CameraPositionPlugin)

} // namespace oculus_gz_navigator
