using UnityEngine;
using UnityEngine.Rendering;
using System.IO;
using ROS2;

// @TODO:
// . support custom color wheels in optical flow via lookup textures
// . support custom depth encoding
// . support multiple overlay cameras
// . tests
// . better example scene(s)

// @KNOWN ISSUES
// . Motion Vectors can produce incorrect results in Unity 5.5.f3 when
//      1) during the first rendering frame
//      2) rendering several cameras with different aspect ratios - vectors do stretch to the sides of the screen

[RequireComponent(typeof(Camera))]
public class DepthCamera : Sensor<sensor_msgs.msg.Image>
{
    Camera cam_;
    public Shader uberReplacementShader;
    public int image_width = 320;
    public int image_height = 240;
    sensor_msgs.msg.Image msg;
    byte[] data;
    bool updated_data = false;

    void Start()
    {
        msg = new sensor_msgs.msg.Image();
        ROS2UnityComponent ros2Component = gameObject.GetComponent<ROS2UnityComponent>();
        if (ros2Component == null)
        {
            ros2Component = gameObject.AddComponent<ROS2UnityComponent>();
        }
        ROS2Node ros2Node = ros2Component.CreateNode("UnityDepthImagePub");
        CreateROSParticipants(ros2Component, ros2Node, "");

        cam_ = GetComponent<Camera>();
        // default fallbacks, if shaders are unspecified
        if (!uberReplacementShader)
            uberReplacementShader = Shader.Find("Hidden/UberReplacement");
    }
    protected override void OnUpdate()
    {
        SetupCameraWithReplacementShader(cam_, uberReplacementShader, Color.black);
        UpdateData(cam_, image_width, image_height, false, false);
    }
    protected override sensor_msgs.msg.Image AcquireValue()
    {
        msg.Height = (uint)image_height;
        msg.Width = (uint)image_width;
        msg.Encoding = "bgr8";
        msg.Is_bigendian = 0;
        msg.Step = 3 * msg.Width;
        msg.Data = data;

        return msg;
    }

    protected override bool HasNewData()
    {
        if (data != null && updated_data)
        {
            updated_data = false;
            return true;
        }
        return false;
    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, Color clearColor)
    {
        cam.SetReplacementShader(shader, "");
        cam.backgroundColor = clearColor;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.allowHDR = false;
        cam.allowMSAA = false;
    }

    private void UpdateData(Camera cam, int width, int height, bool supportsAntialiasing, bool needsRescale)
    {
        var mainCamera = GetComponent<Camera>();
        var depth = 24;
        var format = RenderTextureFormat.Default;
        var readWrite = RenderTextureReadWrite.Default;
        var antiAliasing = (supportsAntialiasing) ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

        var finalRT =
            RenderTexture.GetTemporary(width, height, depth, format, readWrite, antiAliasing);
        var renderRT = (!needsRescale) ? finalRT :
            RenderTexture.GetTemporary(mainCamera.pixelWidth, mainCamera.pixelHeight, depth, format, readWrite, antiAliasing);
        var tex = new Texture2D(width, height, TextureFormat.RGB24, false);

        var prevActiveRT = RenderTexture.active;
        var prevCameraRT = cam.targetTexture;

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        cam.targetTexture = renderRT;

        cam.Render();

        if (needsRescale)
        {
            // blit to rescale (see issue with Motion Vectors in @KNOWN ISSUES)
            RenderTexture.active = finalRT;
            Graphics.Blit(renderRT, finalRT);
        }

        // read offsreen texture contents into the CPU readable texture
        tex.ReadPixels(new Rect(0, 0, tex.width, tex.height), 0, 0);
        tex.Apply();
        byte[] bytes = tex.GetRawTextureData();
        data = RotateImage180(bytes, image_width, image_height);
       
        // restore state and cleanup
        cam.targetTexture = prevCameraRT;
        RenderTexture.active = prevActiveRT;

        Object.Destroy(tex);
        RenderTexture.ReleaseTemporary(finalRT);
        RenderTexture.ReleaseTemporary(renderRT);
        updated_data = true;
    }

    private byte[] RotateImage180(byte[] bytes, int width, int height)
    {
        byte[] rotatedBytes = new byte[bytes.Length];

        int i = 0;
        for (int y = height - 1; y >= 0; y--)
        {
            for (int x = width - 1; x >= 0; x--)
            {
                int index = (y * width + x) * 3;
                rotatedBytes[i++] = bytes[index + 2];
                rotatedBytes[i++] = bytes[index + 1];
                rotatedBytes[i++] = bytes[index];
            }
        }

        return rotatedBytes;
    }
}
