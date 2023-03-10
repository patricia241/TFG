#include "oculus_gz_navigator/camera_headset_plugin.hpp"

namespace oculus_gz_navigator
{
    /**
     * Implemented virtual function inherited from ModelPlugin
     * 
     * Called when a Plugin is first created, and after the World has been loaded. This function should not be blocking.
     * Save base_link link and create subscription to /camera_position topic.
     * 
     * @param _model Pointer to the Model
     * @param _sdf 	Pointer to the SDF element of the plugin.
    */
    void CameraHeadsetPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        std::cout << "-----------------------------------------" << std::endl;
        std::cout << "CAMERA HEADSET PLUGIN" << std::endl;
        std::cout << "-----------------------------------------" << std::endl;
        std::cout << std::endl;

        link_ = _model->GetLink("base_link");

        // ROS node
        node_ = gazebo_ros::Node::Get(_sdf);

        // ROS subscription
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>("camera_position", 1,
                                                                            std::bind(&CameraHeadsetPlugin::camera_position_callback,
                                                                                    this, std::placeholders::_1));
    }
    
    /**
     * Subscription topic callback
     * 
     * Get new local position for thw camera. Change reference system of this position and set it to 
     * the object camera in Gazebo. Also set the linear and angular speeds of the object to ensure that after
     * changing the position of the object it will stay still (the camera has 0 gravity).
     * 
     * @param msg Received message through /camera_position topic
    */
    void CameraHeadsetPlugin::camera_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        ignition::math::Vector3d speeds(0, 0, 0);
        ignition::math::Pose3d pose(msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        
        link_->SetWorldPose(pose);
        link_->SetLinearVel(speeds);
        link_->SetAngularVel(speeds); 
    }

} // namespace oculus_gz_navigator
