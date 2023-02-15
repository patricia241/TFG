using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class InputOculusDevices : MonoBehaviour
{
    public InputDevice right_controller_;

    // Update is called once per frame
    void Update()
    {
        if (!right_controller_.isValid)
        {
            InitializeInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right, ref right_controller_);
        }
    }

    private void InitializeInputDevice(InputDeviceCharacteristics inputCharacteristics, ref InputDevice inputDevice)
    {
        List<InputDevice> devices = new List<InputDevice>();
        // Call InputDevices to see if it can find any devices with characteristics we're looking for
        InputDevices.GetDevicesWithCharacteristics(inputCharacteristics, devices);

        // Our hands might not be active and so they will not be generated from the search
        // Check if any devices are found 
        if (devices.Count > 0)
        {
            inputDevice = devices[0];
        }

    }
}
