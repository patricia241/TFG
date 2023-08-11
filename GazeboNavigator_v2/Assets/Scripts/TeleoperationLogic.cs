using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

public class TeleoperationLogic : MonoBehaviour
{
    ROS2UnityComponent ros_component_;
    private ROS2Node ros_node_;
    private IPublisher<geometry_msgs.msg.Twist> cmd_vel_pub_;
    private int linear_vel_multiplier = 1;
    private int angular_vel_multiplier = 1;

    public string topic = "cmd_vel";
    // Start is called before the first frame update
    void Awake()
    {
        ros_component_ = GetComponent<ROS2UnityComponent>();
    }
    
    private void SendVel(float linear_vel, float angular_vel)
    {
        geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist();
        msg.Linear.X = linear_vel;
        msg.Angular.Z = angular_vel;
        cmd_vel_pub_.Publish(msg);
    }

    private void TeleoperationManager()
    {
        // Return value in range -1.0f to 1.0f on each axis
        Vector2 lft_thumstick_val = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick);
        Vector2 rght_thumstick_val = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick);
        
        // If press X increase linear vel, Y for decrease
        if (OVRInput.GetDown(OVRInput.Button.Four)) {
            linear_vel_multiplier++;
        } else if (OVRInput.GetDown(OVRInput.Button.Three)) {
            linear_vel_multiplier = (linear_vel_multiplier > 1) ? linear_vel_multiplier - 1 : linear_vel_multiplier;
        }
        // If press B increase angular vel, A for decrease
        if (OVRInput.GetDown(OVRInput.Button.Two))
        {
            angular_vel_multiplier++;
        }
        else if (OVRInput.GetDown(OVRInput.Button.One))
        {
            angular_vel_multiplier = (angular_vel_multiplier > 1) ? angular_vel_multiplier - 1 : angular_vel_multiplier;
        }

        SendVel(lft_thumstick_val.y * linear_vel_multiplier, rght_thumstick_val.x * angular_vel_multiplier);
    }

    // Update is called once per frame
    void Update()
    {
        if (ros_component_.Ok())
        {
            if (ros_node_ == null)
            {
                ros_node_ = ros_component_.CreateNode("TeleoperationPubNode");
                cmd_vel_pub_ = ros_node_.CreatePublisher<geometry_msgs.msg.Twist>(topic);
            }
            TeleoperationManager();
        }
    }
}
