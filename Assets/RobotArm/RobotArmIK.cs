using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteInEditMode]
public class RobotArmIK : MonoBehaviour
{
    [Header("IK References")]
    public RobotArm robotArm;
    public Transform target;
    
    [Header("IK Configuration")]
    [SerializeField] private float learningRate = 0.5f;
    [SerializeField] private int maxIterations = 200;
    [SerializeField] private float tolerance = 0.01f;
    [HideInInspector]public bool enableIK = false;
    
    [Header("Advanced IK Settings")]
    [SerializeField] private bool useAdaptiveLearningRate = true;
    [SerializeField] private bool useMomentum = true;
    [SerializeField] private float momentumFactor = 0.8f;
    
    [Header("Editor Settings")]
    [SerializeField] private float updateFrequency = 0.02f;
    
    private float currentError;
    
    private float lastUpdateTime;
    private float[] previousAngleUpdates;
    private Vector3 lastTargetPosition;
    private bool targetMoved = false;
    
#if UNITY_EDITOR
    private double lastEditorUpdateTime;
#endif
    
    void OnEnable()
    {
        previousAngleUpdates = new float[robotArm.JointCount * 3];
    }
    
    void Update()
    {
        if (!enableIK || robotArm == null || target == null) 
            return;
        
        // если таргет двигался
        targetMoved = Vector3.Distance(target.position, lastTargetPosition) > 0.01f;
        lastTargetPosition = target.position;
        
        bool shouldUpdate = targetMoved;
        
        if (Application.isPlaying)
        {
            shouldUpdate = targetMoved || (Time.time - lastUpdateTime >= updateFrequency);
        }
#if UNITY_EDITOR
        else
        {
            double currentTime = EditorApplication.timeSinceStartup;
            shouldUpdate = targetMoved || (currentTime - lastEditorUpdateTime >= updateFrequency);
            if (shouldUpdate) lastEditorUpdateTime = currentTime;
        }
#endif
        
        if (shouldUpdate)
        {
            lastUpdateTime = Time.time;
            SolveIK();
        }
    }
    
    private float CalculateTotalArmLength()
    {
        if (robotArm == null) 
            return 0f;
        
        RobotArmManager manager = robotArm.robotArmManager;
        if (manager != null)
        {
            return manager.SegmentLength * (manager.JointCount - 1);
        }
        
        float totalLength = 0f;
        for (int i = 0; i < robotArm.JointCount - 1; i++)
        {
            Vector3 currentPos = robotArm.GetJointPosition(i);
            Vector3 nextPos = robotArm.GetJointPosition(i + 1);
            totalLength += Vector3.Distance(currentPos, nextPos);
        }
        
        return totalLength;
    }
    
    private void SolveIK()
    {
        if (robotArm == null || target == null) return;
        if (robotArm.JointCount == 0) return;

        Vector3 targetPosition = target.position;
        Vector3 endEffectorPos = robotArm.GetEndEffectorPosition();
        
        float initialDistance = Vector3.Distance(endEffectorPos, targetPosition);
        currentError = initialDistance;

        // цель поймана
        if (initialDistance <= tolerance && !targetMoved) 
            return;
        
        // рассчитываем максимальную достижимую дистанцию
        float maxReach = CalculateTotalArmLength();
        Vector3 basePosition = GetArmBasePosition();
        Vector3 baseToTarget = targetPosition - basePosition;
        float distanceToTarget = baseToTarget.magnitude;

        // если цель вне досягаемости, проецируем ее на сферу достижимости
        if (distanceToTarget > maxReach)
        {
            targetPosition = basePosition + baseToTarget.normalized * maxReach;
            currentError = Vector3.Distance(endEffectorPos, targetPosition);
        }

        // несколько стратегий
        float[] bestAngles = SolveIKWithMultipleStrategies(targetPosition, initialDistance);
        SetAngles(bestAngles);
        
        // пересчет ошибки 
        currentError = Vector3.Distance(robotArm.GetEndEffectorPosition(), targetPosition);
    }

    private float[] SolveIKWithMultipleStrategies(Vector3 targetPosition, float initialError)
    {
        float[] initialAngles = GetCurrentAngles();
        float[] bestAngles = (float[])initialAngles.Clone();
        float bestError = initialError;

        // стандартный градиентный спуск
        float error1 = SolveWithGradientDescent(targetPosition, initialAngles.Clone() as float[]);
        if (error1 < bestError)
        {
            bestError = error1;
            bestAngles = GetCurrentAngles();
        }

        // CCD-like подход (для длинных цепей)
        SetAngles(initialAngles); // начальные углы
        float error2 = SolveWithCCDApproach(targetPosition);
        if (error2 < bestError)
        {
            bestError = error2;
            bestAngles = GetCurrentAngles();
        }

        // адаптивный learning rate для сложных случаев
        if (bestError > tolerance * 2f)
        {
            SetAngles(initialAngles);
            float error3 = SolveWithAdaptiveStrategy(targetPosition);
            if (error3 < bestError)
            {
                bestAngles = GetCurrentAngles();
            }
        }

        return bestAngles;
    }
    
    private float SolveWithGradientDescent(Vector3 targetPosition, float[] angles)
    {
        SetAngles(angles);
        float currentError = Vector3.Distance(robotArm.GetEndEffectorPosition(), targetPosition);
        
        for (int i = 0; i < maxIterations; i++)
        {
            for (int jointIndex = 0; jointIndex < robotArm.JointCount; jointIndex++)
            {
                UpdateJointWithGradient(jointIndex, targetPosition, learningRate);
            }
            
            float newError = Vector3.Distance(robotArm.GetEndEffectorPosition(), targetPosition);
            if (newError <= tolerance) break;
            currentError = newError;
        }
        
        return currentError;
    }
    
    private void UpdateJointWithGradient(int jointIndex, Vector3 targetPosition, float currentLR)
    {
        JointConfig jointConfig = robotArm.GetJointConfig(jointIndex);
        if (jointConfig == null) return;

        float deltaAngle = 0.2f;
        
        TryUpdateAxis(jointIndex, JointAxis.X, jointConfig.xAxis, deltaAngle, targetPosition, currentLR);
        TryUpdateAxis(jointIndex, JointAxis.Y, jointConfig.yAxis, deltaAngle, targetPosition, currentLR);
        TryUpdateAxis(jointIndex, JointAxis.Z, jointConfig.zAxis, deltaAngle, targetPosition, currentLR);
    }

    private float SolveWithCCDApproach(Vector3 targetPosition)
    {
        float currentError = Vector3.Distance(robotArm.GetEndEffectorPosition(), targetPosition);
        
        for (int iteration = 0; iteration < maxIterations / 2; iteration++)
        {
            // от концевого джоинта к началу
            for (int jointIndex = robotArm.JointCount - 1; jointIndex >= 0; jointIndex--)
            {
                UpdateJointWithCCD(jointIndex, targetPosition);
            }
            
            float newError = Vector3.Distance(robotArm.GetEndEffectorPosition(), targetPosition);
            if (newError <= tolerance) break;
            if (Mathf.Abs(newError - currentError) < 0.0001f) break;
            
            currentError = newError;
        }
        
        return currentError;
    }

    private void UpdateJointWithCCD(int jointIndex, Vector3 targetPosition)
    {
        JointConfig jointConfig = robotArm.GetJointConfig(jointIndex);
        if (jointConfig == null) return;

        Vector3 jointPos = robotArm.GetJointPosition(jointIndex);
        Vector3 endEffectorPos = robotArm.GetEndEffectorPosition();
        
        //получаем текущие направления
        Vector3 toEndEffector = (endEffectorPos - jointPos).normalized;
        Vector3 toTarget = (targetPosition - jointPos).normalized;

        // для каждой активной оси в инспекторе вычисляем необходимый поворот
        if (jointConfig.xAxis.enabled)
        {
            float angle = CalculateCCDAngle(jointPos, toEndEffector, toTarget, Vector3.right, jointConfig.xAxis);
            if (Mathf.Abs(angle) > 0.01f)
            {
                float newAngle = jointConfig.xAxis.currentAngle + angle * learningRate * 2f;
                jointConfig.xAxis.currentAngle = Mathf.Clamp(newAngle, jointConfig.xAxis.minAngle, jointConfig.xAxis.maxAngle);
            }
        }
        
        if (jointConfig.yAxis.enabled)
        {
            float angle = CalculateCCDAngle(jointPos, toEndEffector, toTarget, Vector3.up, jointConfig.yAxis);
            if (Mathf.Abs(angle) > 0.1f)
            {
                float newAngle = jointConfig.yAxis.currentAngle + angle * learningRate * 2f;
                jointConfig.yAxis.currentAngle = Mathf.Clamp(newAngle, jointConfig.yAxis.minAngle, jointConfig.yAxis.maxAngle);
            }
        }
        
        if (jointConfig.zAxis.enabled)
        {
            float angle = CalculateCCDAngle(jointPos, toEndEffector, toTarget, Vector3.forward, jointConfig.zAxis);
            if (Mathf.Abs(angle) > 0.1f)
            {
                float newAngle = jointConfig.zAxis.currentAngle + angle * learningRate * 2f;
                jointConfig.zAxis.currentAngle = Mathf.Clamp(newAngle, jointConfig.zAxis.minAngle, jointConfig.zAxis.maxAngle);
            }
        }
    }

    private float CalculateCCDAngle(Vector3 jointPos, Vector3 toEndEffector, Vector3 toTarget, Vector3 axis, JointAxisConfig axisConfig)
    {
        // проекции векторов на плоскость, перпендикулярную оси вращения
        Vector3 projToEnd = Vector3.ProjectOnPlane(toEndEffector, axis).normalized;
        Vector3 projToTarget = Vector3.ProjectOnPlane(toTarget, axis).normalized;
        
        if (projToEnd.magnitude < 0.001f || projToTarget.magnitude < 0.001f)
            return 0f;
            
        float angle = Vector3.SignedAngle(projToEnd, projToTarget, axis);
        return Mathf.Clamp(angle, -5f, 5f); // Ограничиваем максимальный шаг
    }

    private float SolveWithAdaptiveStrategy(Vector3 targetPosition)
    {
        float originalLearningRate = learningRate;
        float currentError = Vector3.Distance(robotArm.GetEndEffectorPosition(), targetPosition);
        
        for (int iteration = 0; iteration < maxIterations / 3; iteration++)
        {
            float adaptiveLR = originalLearningRate * Mathf.Lerp(0.5f, 2f, currentError / CalculateTotalArmLength());
            
            // случайный выбор порядка обновления суставов
            // для выхода из локальных минимумов
            if (iteration % 10 == 0)
            {
                UpdateJointsInRandomOrder(targetPosition, adaptiveLR);
            }
            else
            {
                UpdateJointsInCCDOrder(targetPosition, adaptiveLR);
            }
            
            float newError = Vector3.Distance(robotArm.GetEndEffectorPosition(), targetPosition);
            if (newError <= tolerance) break;
            
            if (newError >= currentError - 0.001f)
            {
                learningRate = originalLearningRate * 0.5f;
            }
            
            currentError = newError;
        }
        
        learningRate = originalLearningRate;
        return currentError;
    }

    private void UpdateJointsInRandomOrder(Vector3 targetPosition, float adaptiveLR)
    {
        // случайный порядок обновления суставов для выхода из локальных минимумов
        List<int> indices = new List<int>();
        for (int i = 0; i < robotArm.JointCount; i++) indices.Add(i);
        
        // перемешиваем индексы
        for (int i = 0; i < indices.Count; i++)
        {
            int randomIndex = Random.Range(i, indices.Count);
            int temp = indices[i];
            indices[i] = indices[randomIndex];
            indices[randomIndex] = temp;
        }
        
        foreach (int jointIndex in indices)
        {
            UpdateJointWithGradient(jointIndex, targetPosition, adaptiveLR);
        }
    }
    
    private void UpdateJointsInCCDOrder(Vector3 targetPosition, float adaptiveLR)
    {
        for (int jointIndex = robotArm.JointCount - 1; jointIndex >= 0; jointIndex--)
        {
            UpdateJointWithGradient(jointIndex, targetPosition, adaptiveLR);
        }
    }
    
   
    private void TryUpdateAxis(int jointIndex, JointAxis axis, JointAxisConfig axisConfig, 
                             float deltaAngle, Vector3 targetPosition, float adaptiveLR)
    {
        if (!axisConfig.enabled) return;
        
        float originalAngle = axisConfig.currentAngle;
        
        // градиент расчитывается через центральные разности
        float gradient = CalculateCentralDifferenceGradient(jointIndex, axis, axisConfig, deltaAngle, targetPosition);
        
        // применение инерции для сходимости
        int updateIndex = jointIndex * 3 + (int)axis;
        float update = adaptiveLR * gradient;
        
        if (useMomentum && previousAngleUpdates != null && updateIndex < previousAngleUpdates.Length)
        {
            update += momentumFactor * previousAngleUpdates[updateIndex];
            previousAngleUpdates[updateIndex] = update;
        }
        
        // новые углы
        float updatedAngle = originalAngle - update;
        updatedAngle = Mathf.Clamp(updatedAngle, axisConfig.minAngle, axisConfig.maxAngle);
        
        robotArm.SetJointAngle(jointIndex, axis, updatedAngle);
    }
    
    private float CalculateCentralDifferenceGradient(int jointIndex, JointAxis axis, 
                                                   JointAxisConfig axisConfig, float deltaAngle, Vector3 targetPosition)
    {
        float originalAngle = axisConfig.currentAngle;
        Vector3 originalPos = robotArm.GetEndEffectorPosition();
        float originalError = Vector3.Distance(originalPos, targetPosition);
        
        // положительное возмущение угла
        float anglePlus = Mathf.Clamp(originalAngle + deltaAngle, axisConfig.minAngle, axisConfig.maxAngle);
        robotArm.SetJointAngle(jointIndex, axis, anglePlus);
        Vector3 posPlus = robotArm.GetEndEffectorPosition();
        float errorPlus = Vector3.Distance(posPlus, targetPosition);
        
        // отрицательное возмущение 
        float angleMinus = Mathf.Clamp(originalAngle - deltaAngle, axisConfig.minAngle, axisConfig.maxAngle);
        robotArm.SetJointAngle(jointIndex, axis, angleMinus);
        Vector3 posMinus = robotArm.GetEndEffectorPosition();
        float errorMinus = Vector3.Distance(posMinus, targetPosition);
        
        robotArm.SetJointAngle(jointIndex, axis, originalAngle);
        
        // метод центральных разностей
        if (Mathf.Abs(errorPlus - errorMinus) < 0.0001f) 
            return 0f;
        
        return (errorPlus - errorMinus) / (2f * deltaAngle);
    }
    
    private Vector3 GetArmBasePosition()
    {
        if (robotArm == null || robotArm.JointCount == 0) return Vector3.zero;
    
        if (robotArm.robotArmManager != null)
        {
            return robotArm.robotArmManager.transform.TransformPoint(robotArm.robotArmManager.StartPosition);
        }
    
        return robotArm.GetJointPosition(0);
    }
    
    private float[] GetCurrentAngles()
    {
        if (robotArm == null) return new float[0];
        
        float[] angles = new float[robotArm.JointCount * 3];
        
        for (int i = 0; i < robotArm.JointCount; i++)
        {
            JointConfig config = robotArm.GetJointConfig(i);
            if (config != null)
            {
                angles[i * 3] = config.xAxis.currentAngle;
                angles[i * 3 + 1] = config.yAxis.currentAngle;
                angles[i * 3 + 2] = config.zAxis.currentAngle;
            }
        }
        
        return angles;
    }
    
    private void SetAngles(float[] angles)
    {
        if (robotArm == null || angles == null) return;
        
        for (int i = 0; i < robotArm.JointCount; i++)
        {
            if (i * 3 + 2 < angles.Length)
            {
                robotArm.SetJointAngle(i, JointAxis.X, angles[i * 3]);
                robotArm.SetJointAngle(i, JointAxis.Y, angles[i * 3 + 1]);
                robotArm.SetJointAngle(i, JointAxis.Z, angles[i * 3 + 2]);
            }
        }
    }
   
}