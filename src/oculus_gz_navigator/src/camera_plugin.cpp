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

            link_ = _model->GetLink("base_link");

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

        ignition::math::Matrix3<double> generate_matrix3_from_point(double x, double y, double z) {
          ignition::math::Matrix3<double> point_mat(0, 0, x, 
                                                    0, 0, y,
                                                    0, 0, z);
          return point_mat;
        }

        ignition::math::Matrix3<double> get_rotation_mat(double roll, double pitch, double yaw) {
          ignition::math::Matrix3<double> rotx_mat(1, 0, 0, 
                                                   0, cos(roll), -sin(roll),
                                                   0, sin(roll), cos(roll));
          ignition::math::Matrix3<double> roty_mat(cos(pitch), 0, sin(pitch), 
                                                   0, 1, 0,
                                                   -sin(pitch), 0, cos(pitch));
          ignition::math::Matrix3<double> rotz_mat(cos(yaw), -sin(yaw), 0, 
                                                   sin(yaw), cos(yaw), 0,
                                                   0, 0, 1);                                        
          return rotz_mat * roty_mat * rotx_mat;
        }

        ignition::math::Matrix3<double> generate_change_base_matrix() {
          ignition::math::Pose3d current_pose = link_->WorldPose();
          ignition::math::Matrix3<double> axis_x_end = generate_matrix3_from_point(1, 0, 0);
          ignition::math::Matrix3<double> axis_y_end = generate_matrix3_from_point(0, 1, 0);
          ignition::math::Matrix3<double> axis_z_end = generate_matrix3_from_point(0, 0, 1);
          ignition::math::Matrix3<double> position = generate_matrix3_from_point(current_pose.X(), current_pose.Y(), current_pose.Z());
          ignition::math::Matrix3<double> rot_mat = get_rotation_mat(current_pose.Roll(), current_pose.Pitch(), current_pose.Yaw());

          // DEBUG
          std::cout << "MATRIZ ROTACIÓN: " << std::endl;
          std::cout << rot_mat(0, 0) << " " <<  rot_mat(0, 1) << " " << rot_mat(0, 2) << std::endl;
          std::cout << rot_mat(1, 0) << " " <<  rot_mat(1, 1) << " " << rot_mat(1, 2) << std::endl;
          std::cout << rot_mat(2, 0) << " " <<  rot_mat(2, 1) << " " << rot_mat(2, 2) << std::endl;

          // Rotated and translated axis
          ignition::math::Matrix3<double> rt_axis_x_end = rot_mat * axis_x_end + position;
          ignition::math::Matrix3<double> rt_axis_y_end = rot_mat * axis_y_end + position;
          ignition::math::Matrix3<double> rt_axis_z_end = rot_mat * axis_z_end + position;

          ignition::math::Matrix3<double> change_base_mat(rt_axis_x_end(0, 2), rt_axis_y_end(0, 2), rt_axis_z_end(0, 2),
                                                          rt_axis_x_end(1, 2), rt_axis_y_end(1, 2), rt_axis_z_end(1, 2),
                                                          rt_axis_x_end(2, 2), rt_axis_y_end(2, 2), rt_axis_z_end(2, 2));


          // DEBUG
          std::cout << "MATRIZ CAMBIO DE BASE: " << std::endl;
          std::cout << change_base_mat(0, 0) << " " <<  change_base_mat(0, 1) << " " << change_base_mat(0, 2) << std::endl;
          std::cout << change_base_mat(1, 0) << " " <<  change_base_mat(1, 1) << " " << change_base_mat(1, 2) << std::endl;
          std::cout << change_base_mat(2, 0) << " " <<  change_base_mat(2, 1) << " " << change_base_mat(2, 2) << std::endl;

          return change_base_mat;
        }
        // If in msg Position I get (0, 0, 1) increment z position 1m. Rotation set directly with msg info. 
        void camera_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
          ignition::math::Vector3d speeds(0, 0, 0);
          ignition::math::Pose3d pose = link_->WorldPose();

          ignition::math::Matrix3 point_global = generate_change_base_matrix() * generate_matrix3_from_point(msg->position.x, msg->position.y, msg->position.z);
          ignition::math::Vector3d final_result(point_global(0, 2), point_global(1, 2), point_global(2, 2));
          // ignition::math::Quaternion orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
          
          std::cout << "X: " << final_result.X() << " Y: " << final_result.Y() << " Z: " << final_result.Z() << std::endl;
          pose.Set(final_result, pose.Rot());

          link_->SetWorldPose(pose);
          link_->SetLinearVel(speeds);
          link_->SetAngularVel(speeds); 
        }
  };

  GZ_REGISTER_MODEL_PLUGIN(CameraPositionPlugin)

} // namespace oculus_gz_navigator
