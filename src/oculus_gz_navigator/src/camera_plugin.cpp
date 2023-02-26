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

        void multiply_vector_matrix(ignition::math::Vector4<float> point, ignition::math::Matrix4<float> tr_mat, ignition::math::Vector4<float>& result) {
          for(int i = 0;  i < 4; i++){
            for (int j = 0; j < 4; j++){
              result[i] += tr_mat(i, j) * point[j];
            }
          }
        }

        /*
          @local_pose: variable que indica los metros que se quiere mover la cámara en X, Y, Z tomando como sistema de referencias la cámara
          @global_pose: es dónde se guardará el resultado, las coordenadas respecto al sistema de referencias del mundo dónde habrá que situar la cámara
        */
        void get_transformed_coordinate(ignition::math::Vector4<float> local_pose, ignition::math::Vector4<float>& global_pose) {
          ignition::math::Vector4<float> traslation_vector;
          ignition::math::Pose3d pose = link_->WorldPose();
          float roll = pose.Rot().Roll();
          float pitch = pose.Rot().Pitch();
          float yaw = pose.Rot().Yaw();

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
          // Roto local_pose según los ángulos que esté rotada mi cámara
          ignition::math::Matrix4<float> rot_mat = rotation_x_mat * rotation_y_mat * rotation_z_mat;
          multiply_vector_matrix(local_pose, rot_mat, traslation_vector);
          
          // Traslado la cámara
          ignition::math::Vector4<float> current_pose(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), 1);
          global_pose = current_pose + traslation_vector;          
        }

        // If in msg Position I get (0, 0, 1) increment z position 1m. Rotation set directly with msg info. 
        void camera_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
          ignition::math::Vector3d speeds(0, 0, 0); 
          ignition::math::Pose3d pose = link_->WorldPose();
          ignition::math::Vector4<float> global_pose;
          ignition::math::Vector4<float> local_pose(msg->position.x, msg->position.y, msg->position.z, 1);

          get_transformed_coordinate(local_pose, global_pose);
          
          pose.SetX(global_pose.X());
          pose.SetY(global_pose.Y());
          pose.SetZ(global_pose.Z());
          
          link_->SetWorldPose(pose);
          link_->SetLinearVel(speeds);
          link_->SetAngularVel(speeds); 
        }
  };

  GZ_REGISTER_MODEL_PLUGIN(CameraPositionPlugin)

} // namespace oculus_gz_navigator
