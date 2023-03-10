#include "oculus_gz_navigator/camera_plugin.hpp"

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
    void CameraPositionPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
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
   
    /**
     * Convert positions x, y, z of a point or vector to Matrix3x3
     * 
     * This method places the values passed as arguments in the last column of the Matrix3.
     * This method has been necessary because there is no operator for type ignition::math::Matrix3 that multiplies Matrix3 by Vector3, 
     * but if there exists the operator that multiplies Matrix3 by Matrix3.
     * 
     * @param x Identifies x coord of point or vector
     * @param y Identifies y coord of point or vector
     * @param z Identifies z coord of point or vector
     * 
     * @return Matriz3 with arguments as last column of it
    */
    ignition::math::Matrix3<double> CameraPositionPlugin::generate_matrix3_from_point(double x, double y, double z) {
        ignition::math::Matrix3<double> point_mat(0, 0, x, 
                                                0, 0, y,
                                                0, 0, z);
        return point_mat;
    }

    /**
     * Calculate extrinsic rotation matrix
     * 
     * Given the orientations in radians, calculate the extrinsic rotation matrix, 
     * the order in which the rotations are applied is x, y, z.
     * 
     * @param roll Rotation in x axis (radians)
     * @param pitch Rotation in y axis (radians)
     * @param yaw Rotation in z axis (radians)
     * 
     * @return Extrinsic rotation matrix
    */
    ignition::math::Matrix3<double> CameraPositionPlugin::get_rotation_mat(double roll, double pitch, double yaw) {
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

    /**
     * Calculate base change matrix
     * 
     * Define axis ends of world reference system, rotate and translate each one.
     * From the result obtained from these transformations, the base change matrix is formed.
     * 
     * @return Base change matrix
    */
    ignition::math::Matrix3<double> CameraPositionPlugin::generate_base_change_matrix() {
        ignition::math::Pose3d current_pose = link_->WorldPose();

        // Axis end of world reference system
        ignition::math::Matrix3<double> axis_x_end = generate_matrix3_from_point(1, 0, 0);
        ignition::math::Matrix3<double> axis_y_end = generate_matrix3_from_point(0, 1, 0);
        ignition::math::Matrix3<double> axis_z_end = generate_matrix3_from_point(0, 0, 1);

        // Matrix that contain in its last column current position coordenates
        ignition::math::Matrix3<double> position = generate_matrix3_from_point(current_pose.X(), current_pose.Y(), current_pose.Z());
        ignition::math::Matrix3<double> rot_mat = get_rotation_mat(current_pose.Roll(), current_pose.Pitch(), current_pose.Yaw());

        // Rotated and translated axis
        ignition::math::Matrix3<double> rt_axis_x_end = rot_mat * axis_x_end + position;
        ignition::math::Matrix3<double> rt_axis_y_end = rot_mat * axis_y_end + position;
        ignition::math::Matrix3<double> rt_axis_z_end = rot_mat * axis_z_end + position;

        ignition::math::Matrix3<double> change_base_mat(rt_axis_x_end(0, 2), rt_axis_y_end(0, 2), rt_axis_z_end(0, 2),
                                                        rt_axis_x_end(1, 2), rt_axis_y_end(1, 2), rt_axis_z_end(1, 2),
                                                        rt_axis_x_end(2, 2), rt_axis_y_end(2, 2), rt_axis_z_end(2, 2));
        return change_base_mat;
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
    void CameraPositionPlugin::camera_position_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        ignition::math::Vector3d speeds(0, 0, 0);
        ignition::math::Pose3d pose = link_->WorldPose();

        ignition::math::Matrix3 point_global = generate_base_change_matrix() * generate_matrix3_from_point(msg->position.x, msg->position.y, msg->position.z);
        // ignition::math::Vector3d final_result(point_global(0, 2), point_global(1, 2), point_global(2, 2));
        ignition::math::Vector3d final_result(point_global(0, 2), point_global(1, 2), 1.6);
        
        pose.Set(final_result, pose.Rot());

        link_->SetWorldPose(pose);
        link_->SetLinearVel(speeds);
        link_->SetAngularVel(speeds); 
    }

} // namespace oculus_gz_navigator
