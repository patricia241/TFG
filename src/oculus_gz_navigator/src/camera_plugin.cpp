#include <chrono>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose.hpp"

#define DEAD_ZONE 0.1;

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
            pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>("camera_position", 1,
                                                                              std::bind(&CameraPositionPlugin::camera_position_callback,
                                                                                        this, std::placeholders::_1));
        }
    private:
        gazebo::physics::LinkPtr link_;

        // GazeboROS node 
        gazebo_ros::Node::SharedPtr node_;

        // ROS subscriptor
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

        void get_transformed_coordinate(ignition::math::Vector4<float> local_pose, ignition::math::Vector4<float>& global_pose) {
          ignition::math::Pose3d pose = link_->WorldPose();
          float roll = -pose.Rot().Roll();
          float pitch = -pose.Rot().Pitch();
          float yaw = -pose.Rot().Yaw();

          std::cout << "Angles: " << roll << " " << pitch << " " << yaw << std::endl;

          ignition::math::Matrix4<float> traslation_mat(1, 0, 0, -pose.Pos().X(),
                                                 0, 1, 0, -pose.Pos().Y(),
                                                 0, 0, 1, 0,
                                                 0, 0, 0, 1);
          ignition::math::Matrix4<float> rotation_x_mat(1, 0, 0, 0,
                                                 0, cos(roll), -sin(roll), 0,
                                                 0, sin(roll), cos(roll), 0,
                                                 0, 0, 0, 1);
          ignition::math::Matrix4<float> rotation_y_mat(cos(pitch), 0, -sin(pitch), 0,
                                                 0, 1, 0, 0,
                                                 sin(pitch), 0, cos(pitch), 0,
                                                 0, 0, 0, 1);
          ignition::math::Matrix4<float> rotation_z_mat(cos(yaw), -sin(yaw), 0, 0,
                                                 sin(yaw), cos(yaw), 0, 0,
                                                 0, 0, 1, 0,
                                                 0, 0, 0, 1);

          ignition::math::Matrix4<float> traslation_rot_mat = traslation_mat * rotation_x_mat * rotation_y_mat * rotation_z_mat;
          global_pose = local_pose * traslation_rot_mat;
        }

        // If in msg Position I get (0, 0, 1) increment z position 1m. Rotation set directly with msg info. 
        void camera_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
          ignition::math::Vector3d speeds(0, 0, 0); 
          ignition::math::Pose3d pose = link_->WorldPose();
          /*
          float k = 5.0;
          pose.SetX(pose.Pos().X() + msg->position.x / k);
          pose.SetY(pose.Pos().Y() + msg->position.y / k);
          pose.SetZ(pose.Pos().Z());
          */
          ignition::math::Vector4<float> global_pose;
          ignition::math::Vector4<float> local_pose(msg->position.x/5, msg->position.y/5, 0, 1);

          if (msg->position.x != 0 && msg->position.y != 0) {
            get_transformed_coordinate(local_pose, global_pose);

            pose.SetX(global_pose.X() + pose.Pos().X());
            pose.SetY(global_pose.Y() + pose.Pos().Y());
            pose.SetZ(pose.Pos().Z() + pose.Pos().Z());
            
            std::cout << "Pose X: " << global_pose.X() << " Pose Y: " << global_pose.Y() << std::endl;

            link_->SetWorldPose(pose);
          }
          
          link_->SetLinearVel(speeds);
        }
  };

  GZ_REGISTER_MODEL_PLUGIN(CameraPositionPlugin)

} // namespace oculus_gz_navigator
