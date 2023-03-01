#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose.hpp"


namespace oculus_gz_navigator
{
  class CameraPositionPlugin : public gazebo::ModelPlugin
  {

    public: 
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
        gazebo::physics::LinkPtr link_;
        // GazeboROS node 
        gazebo_ros::Node::SharedPtr node_;
        // ROS subscriptor
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

        ignition::math::Matrix3<double> generate_matrix3_from_point(double x, double y, double z);

        ignition::math::Matrix3<double> get_rotation_mat(double roll, double pitch, double yaw);

        ignition::math::Matrix3<double> generate_base_change_matrix();

        void camera_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  };

  GZ_REGISTER_MODEL_PLUGIN(CameraPositionPlugin)

} // namespace oculus_gz_navigator
