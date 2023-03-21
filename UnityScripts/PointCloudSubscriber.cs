using ROS2;
using System;
using UnityEngine;

public class PointCloudSubscriber : MonoBehaviour
{
    public string topic = "/intel_realsense_r200_depth/points";

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<sensor_msgs.msg.PointCloud2> pcl_sub;

    sensor_msgs.msg.PointField[] fields; // Field of message where I can extract offsets
    uint point_step;  // Length of a point in bytes
    uint n_point_per_row;
    bool msg_received = false;
    bool first_msg = true;
    byte[] pcl_data;

    private Vector3[] pcl_positions;
    private Color[] pcl_colors;

    // Mesh stores the positions and colours of every point in the cloud
    // The renderer and filter are used to display it
    Mesh mesh;
    MeshRenderer meshRenderer;
    MeshFilter mf;

    // The size of pointcloud's points
    public float pointSize = 1f;

    public Transform offset; // Put any gameobject that facilitates adjusting the origin of the pointcloud in VR. 

    void Awake()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        
        // Give all the required components to the gameObject
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        mf = gameObject.AddComponent<MeshFilter>();
        meshRenderer.material = new Material(Shader.Find("Custom/PointCloudShader"));
        mesh = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };

        transform.position = offset.position;
        transform.rotation = offset.rotation;
    }

    void PCLCallback(sensor_msgs.msg.PointCloud2 msg)
    {
        if (first_msg)
        {
            point_step = msg.Point_step;
            n_point_per_row = msg.Row_step / point_step;
            fields = msg.Fields;

            pcl_data = new byte[msg.Row_step];
            pcl_positions = new Vector3[n_point_per_row];
            pcl_colors = new Color[n_point_per_row];

            first_msg = false;
        }
        pcl_data = msg.Data;
        msg_received = true;
    }

    // Update is called once per frame
    void Update()
    {
        if (ros2Unity.Ok() && ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("ROS2UnityCameraNode");
            QualityOfServiceProfile qos = new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);

            pcl_sub = ros2Node.CreateSubscription<sensor_msgs.msg.PointCloud2>(
              topic, PCLCallback, qos);
        }

        if (msg_received)
        {
            PCLRendering();

            transform.position = offset.position;
            transform.rotation = offset.rotation;
            meshRenderer.material.SetFloat("_PointSize", pointSize);
            UpdateMesh();

            msg_received = false;
        }
    }

    void PCLRendering()
    {
        uint x_start_indx, y_start_indx, z_start_indx, rgb_start_indx;
        float x, y, z, r, g, b, rgb_max = 255f; // Values of positions and color channels
        int r_offset = 0, g_offset = 1, b_offset = 2;

        for (uint i = 0; i < n_point_per_row; i++)
        {
            x_start_indx = i * point_step + fields[0].Offset;
            y_start_indx = i * point_step + fields[1].Offset;
            z_start_indx = i * point_step + fields[2].Offset;
            rgb_start_indx = i * point_step + fields[3].Offset;

            // Convert to float 4 bytes from start index. It's doing for each position component
            x = BitConverter.ToSingle(pcl_data, (int) x_start_indx);
            y = BitConverter.ToSingle(pcl_data, (int) y_start_indx);
            z = BitConverter.ToSingle(pcl_data, (int) z_start_indx);

            r = pcl_data[(int)rgb_start_indx + r_offset] / rgb_max;
            g = pcl_data[(int)rgb_start_indx + g_offset] / rgb_max;
            b = pcl_data[(int)rgb_start_indx + b_offset] / rgb_max; 

            pcl_positions[i] = new Vector3(-1f * x, y, z);
            pcl_colors[i] = new Color(r, g, b, 1f);
        } 
    }

    void UpdateMesh()
    {
        mesh.Clear();
        mesh.vertices = pcl_positions;
        mesh.colors = pcl_colors;
        int[] indices = new int[pcl_positions.Length];

        for (int i = 0; i < pcl_positions.Length; i++)
        {
            indices[i] = i;
        }

        mesh.SetIndices(indices, MeshTopology.Points, 0);
        mf.mesh = mesh;
    }
}
