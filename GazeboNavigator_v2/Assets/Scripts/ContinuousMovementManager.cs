using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ContinuousMovementManager : MonoBehaviour
{
    public OVRPlayerController controller_;
    public TeleoperationLogic teleop_logic_;
    private SimpleCapsuleWithStickMovement capsuleMovementComponent_;

    private void Start()
    {
        capsuleMovementComponent_ = controller_.GetComponent<SimpleCapsuleWithStickMovement>();
    }
    public void ScriptManage()
    {
        capsuleMovementComponent_.enabled = !capsuleMovementComponent_.enabled;
        teleop_logic_.gameObject.SetActive(!capsuleMovementComponent_.enabled);
    }
}
