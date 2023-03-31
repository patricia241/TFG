using ROS2;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Threading;
using System.Drawing;
using System.Linq;

public class DiffDrivePlugin : MonoBehaviour
{
    public enum OdomSource
    {
        /// Use an ancoder
        ENCODER = 0,

        /// Use ground truth from simulation world
        WORLD = 1,
    };
    public OdomSource odom_source_;
    
    /// Right wheel
    int RIGHT = 0;

    /// Left wheel
    int LEFT = 1;
    

    public string topic = "/cmd_vel";

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<geometry_msgs.msg.Twist> cmd_vel_sub_;
    private IPublisher<nav_msgs.msg.Odometry> odometry_pub_;

    /// Distance between the wheels, in meters.
    [Tooltip("Número de pares de ruedas del robot")]
    public List<double> wheel_separation_;

    /// Diameter of wheels, in meters.
    [Tooltip("Número de pares de ruedas del robot")]
    public List<double> wheel_diameter_;

    /// Maximum wheel torque, in Nm.
    public double max_wheel_torque_ = 20.0;

    /// Maximum wheel acceleration
    public double max_wheel_accel_ = 1.0;

    /// Desired wheel speed.
    List<double> desired_wheel_speed_;

    /// Speed sent to wheel.
    List<double> wheel_speed_instr_;

    // Nombres de los GameObjects de las ruedas
    public List<string> wheel_names_;

    private List<ArticulationBody> joints_;
    private double target_x_ = 0.0;
    private double target_rot_ = 0.0;

    float last_update_time_;
    /// Protect variables accessed on callbacks.
    Mutex lock_ = new Mutex();

    // Start is called before the first frame update
    void Awake()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        wheel_speed_instr_ = Enumerable.Repeat(0.0, wheel_names_.Count).ToList();
        desired_wheel_speed_ = Enumerable.Repeat(0.0, wheel_names_.Count).ToList();
    }

    void Start()
    {
        joints_ = GetRevoluteJoints(wheel_names_);
    }
    // Update is called once per frame
    void FixedUpdate()
    {
        if (ros2Unity.Ok() && ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("ROS2UnityDiffDriveNode");

            cmd_vel_sub_ = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(
              topic, OnCmdVel);
            odometry_pub_ = ros2Node.CreatePublisher<nav_msgs.msg.Odometry>("odom");
        }

        double seconds_since_last_update = Time.time - last_update_time_;

        UpdateWheelVelocities();
        // Current speed 
        double[] current_speed = new double[wheel_names_.Count];
        int num_wheel_pairs = wheel_names_.Count / 2;
        for (int i = 0; i < num_wheel_pairs; ++i)
        {
            current_speed[2 * i + LEFT] =
              joints_[2 * i + LEFT].jointVelocity[0] * (wheel_diameter_[i] / 2.0);
            current_speed[2 * i + RIGHT] =
              joints_[2 * i + RIGHT].jointVelocity[0] * (wheel_diameter_[i] / 2.0);
        }

        // If max_accel == 0, or target speed is reached
        for (int i = 0; i < num_wheel_pairs; ++i)
        {
            if (max_wheel_accel_ == 0 ||
              ((Math.Abs(desired_wheel_speed_[2 * i + LEFT] - current_speed[2 * i + LEFT]) < 0.01) &&
              (Math.Abs(desired_wheel_speed_[2 * i + RIGHT] - current_speed[2 * i + RIGHT]) < 0.01)))
            {
                joints_[2 * i + LEFT].velocity = 
                    new Vector3(Convert.ToSingle(desired_wheel_speed_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0)), 0, 0);
                joints_[2 * i + RIGHT].velocity = 
                    new Vector3(Convert.ToSingle(desired_wheel_speed_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0)), 0, 0);
            }
            else
            {
                if (desired_wheel_speed_[2 * i + LEFT] >= current_speed[2 * i + LEFT])
                {
                    wheel_speed_instr_[2 * i + LEFT] += Math.Min(
                      desired_wheel_speed_[2 * i + LEFT] -
                      current_speed[2 * i + LEFT], max_wheel_accel_ * seconds_since_last_update);
                }
                else
                {
                    wheel_speed_instr_[2 * i + LEFT] += Math.Max(
                      desired_wheel_speed_[2 * i + LEFT] -
                      current_speed[2 * i + LEFT], -max_wheel_accel_ * seconds_since_last_update);
                }

                if (desired_wheel_speed_[2 * i + RIGHT] > current_speed[2 * i + RIGHT])
                {
                    wheel_speed_instr_[2 * i + RIGHT] += Math.Min(
                      desired_wheel_speed_[2 * i + RIGHT] -
                      current_speed[2 * i + RIGHT], max_wheel_accel_ * seconds_since_last_update);
                }
                else
                {
                    wheel_speed_instr_[2 * i + RIGHT] += Math.Max(
                      desired_wheel_speed_[2 * i + RIGHT] -
                      current_speed[2 * i + RIGHT], -max_wheel_accel_ * seconds_since_last_update);
                }

                joints_[2 * i + LEFT].velocity =
                    new Vector3(Convert.ToSingle(wheel_speed_instr_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0)), 0, 0);
                joints_[2 * i + RIGHT].velocity =
                    new Vector3(Convert.ToSingle(wheel_speed_instr_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0)), 0, 0);
            }
        }
        last_update_time_ = Time.time;
    }

    private void OnCmdVel(geometry_msgs.msg.Twist msg)
    {
        lock_.WaitOne();

        Debug.Log("Velocity" + msg.Linear.X);
        target_x_ = msg.Linear.X;
        target_rot_ = msg.Angular.Z;

        lock_.ReleaseMutex();
    }

    private void UpdateWheelVelocities()
    {
        lock_.WaitOne();

        double vr = target_x_;
        double va = target_rot_;
        int num_wheel_pairs = wheel_names_.Count / 2;

        for(int i = 0; i < num_wheel_pairs; ++i)
        {
            desired_wheel_speed_[2 * i + LEFT] = vr - va * wheel_separation_[i] / 2.0;
            desired_wheel_speed_[2 * i + RIGHT] = vr + va * wheel_separation_[i] / 2.0;
        }

        lock_.ReleaseMutex();
    }
    private List<ArticulationBody> GetRevoluteJoints(List<string> wheel_names_)
    {
        List<ArticulationBody> bodies = new List<ArticulationBody>();

        foreach (string joint_name in wheel_names_)
        {
            // Buscar el GameObject que contiene el ArticulationBody por su nombre
            ArticulationBody wheel = GameObject.Find(joint_name).GetComponent<ArticulationBody>();

            if (wheel.jointType == ArticulationJointType.RevoluteJoint) {
                Debug.Log(wheel.name);
                bodies.Add(wheel);
            } 
        }
        return bodies;
    }
}
