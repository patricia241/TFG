#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Plugin pensado para mandarle posiciones del headset  
namespace oculus_gz_navigator
{
  class CameraHeadsetPlugin : public gazebo::ModelPlugin
  {

    public: 
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
        gazebo::physics::LinkPtr link_;
        // GazeboROS node 
        gazebo_ros::Node::SharedPtr node_;
        // ROS subscriptor
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
        
        void camera_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  };

  GZ_REGISTER_MODEL_PLUGIN(CameraHeadsetPlugin)

} // namespace oculus_gz_navigator
