#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(RobotArm))]
public class RobotArmEditor : Editor
{
    private RobotArm robotArm;
    
    public override void OnInspectorGUI()
    {
        robotArm = (RobotArm)target;
        
        int originalJointCount = robotArm.JointCount;
        
        DrawDefaultInspector();
        
        GUILayout.Space(10);
        
        if (GUILayout.Button("Reset All Joint Angles"))
        {
           // ResetAllJointAngles();
        }
        
        GUILayout.Space(10);
        
        EditorGUILayout.LabelField("Joint Controls", EditorStyles.boldLabel);
        
        for (int i = 0; i < robotArm.JointCount; i++)
        {
            JointConfig config = robotArm.GetJointConfig(i);
            if (config != null)
            {
                EditorGUILayout.BeginVertical("box");
                
                EditorGUILayout.LabelField($"Joint {i}", EditorStyles.boldLabel);
                
                DrawAxisConfig("X Axis", config.xAxis, (angle) => {
                    robotArm.SetJointAngle(i, JointAxis.X, angle);
                });
                
                DrawAxisConfig("Y Axis", config.yAxis, (angle) => {
                    robotArm.SetJointAngle(i, JointAxis.Y, angle);
                });
                
                DrawAxisConfig("Z Axis", config.zAxis, (angle) => {
                    robotArm.SetJointAngle(i, JointAxis.Z, angle);
                });
                
                EditorGUILayout.EndVertical();
            }
        }
        
        if (GUI.changed)
        {
            EditorUtility.SetDirty(robotArm);
            
        }
    }
    
    private void DrawAxisConfig(string label, JointAxisConfig axisConfig, System.Action<float> setAngleAction)
    {
        EditorGUILayout.BeginVertical("box");
        
        axisConfig.enabled = EditorGUILayout.ToggleLeft(label, axisConfig.enabled);
        
        if (axisConfig.enabled)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Angle Limits", GUILayout.Width(100));
            axisConfig.minAngle = EditorGUILayout.FloatField(axisConfig.minAngle, GUILayout.Width(60));
            EditorGUILayout.LabelField("to", GUILayout.Width(20));
            axisConfig.maxAngle = EditorGUILayout.FloatField(axisConfig.maxAngle, GUILayout.Width(60));
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Current Angle", GUILayout.Width(100));
            float newAngle = EditorGUILayout.Slider(axisConfig.currentAngle, axisConfig.minAngle, axisConfig.maxAngle);
            EditorGUILayout.EndHorizontal();
            
            if (newAngle != axisConfig.currentAngle)
            {
                setAngleAction(newAngle);
            }
        }
        
        EditorGUILayout.EndVertical();
    }
}
#endif