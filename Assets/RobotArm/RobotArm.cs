using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class RobotArm : MonoBehaviour
{
    [Header("Arm References")]
    public RobotArmManager robotArmManager;
    public Transform target;
    
    [Header("FK Settings")]
    [HideInInspector]public bool enableFK = true;
    [SerializeField] private float updateFrequency = 0.02f;
    
    private float _lastUpdateTime;
#if UNITY_EDITOR
    private double _lastEditorUpdateTime;
#endif
    
    //сколько джоитов
    public int JointCount 
    { 
        get
        {
            return robotArmManager.JointCount;
        } 
    }
    
    public List<JointConfig> JointConfigs 
    { 
        get
        {
            return robotArmManager.JointConfigs;
        } 
    }

    void OnEnable()
    {
    }
    
    void Update()
    {
        if (!enableFK) return;
        
        bool shouldUpdate = false;
        
        if (Application.isPlaying)
        {
            shouldUpdate = Time.time - _lastUpdateTime >= updateFrequency;
        }
#if UNITY_EDITOR
        else
        {
            double currentTime = UnityEditor.EditorApplication.timeSinceStartup;
            shouldUpdate = currentTime - _lastEditorUpdateTime >= updateFrequency;
            if (shouldUpdate) _lastEditorUpdateTime = currentTime;
        }
#endif
        
        if (shouldUpdate)
        {
            _lastUpdateTime = Time.time;
            UpdateKinematics();
        }
    }
    
    public void UpdateKinematics()
    {
        if (robotArmManager != null && robotArmManager.VisualsInitialized)
        {
            robotArmManager.ApplyKinematicsToVisuals();
        }
    }
    
    //применяем изменения в инспекторе
    public void SetJointAngle(int jointIndex, JointAxis axis, float angle)
    {
        var configs = JointConfigs;
        if (configs != null && jointIndex >= 0 && jointIndex < configs.Count)
        {
            JointConfig config = configs[jointIndex];
            config.SetAxisAngle(axis, angle);
            UpdateKinematics();
        }
    }
    
    public float GetJointAngle(int jointIndex, JointAxis axis)
    {
        var configs = JointConfigs;
        if (configs != null && jointIndex >= 0 && jointIndex < configs.Count)
        {
            return configs[jointIndex].GetAxisAngle(axis);
        }
        return 0f;
    }
    
    public JointConfig GetJointConfig(int jointIndex)
    {
        var configs = JointConfigs;
        if (configs != null && jointIndex >= 0 && jointIndex < configs.Count)
        {
            return configs[jointIndex];
        }
        return null;
    }
    
    public Vector3 GetEndEffectorPosition()
    {
        if (robotArmManager == null || !robotArmManager.VisualsInitialized) 
            return Vector3.zero;
        
        var jointContainers = robotArmManager.JointContainers;
        
        if (jointContainers.Count > 0)
        {
            Transform lastJoint = jointContainers[jointContainers.Count - 1];
            
            return lastJoint.position;
        }
        
        return Vector3.zero;
    }
    
    
    // позиция конкретного сустава
    public Vector3 GetJointPosition(int jointIndex)
    {
        if (robotArmManager == null || !robotArmManager.VisualsInitialized || jointIndex < 0) 
            return Vector3.zero;
            
        var jointContainers = robotArmManager.JointContainers;
        if (jointIndex >= jointContainers.Count) return Vector3.zero;
            
        return jointContainers[jointIndex].position;
    }
}