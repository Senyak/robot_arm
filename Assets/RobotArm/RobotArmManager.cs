using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

[System.Serializable]
public class JointAxisConfig
{
    public float minAngle = -120f;
    public float maxAngle = 120f;

    public float currentAngle = 0f;
    
    public bool enabled = true;
}

[System.Serializable]
public class JointConfig
{
    [Header("X Axis Rotation")]
    public JointAxisConfig xAxis = new JointAxisConfig();
    
    [Header("Y Axis Rotation")]
    public JointAxisConfig yAxis = new JointAxisConfig();
    
    [Header("Z Axis Rotation")]
    public JointAxisConfig zAxis = new JointAxisConfig();
    
    public Quaternion GetRotation()
    {
        Vector3 euler = Vector3.zero;
        
        if (xAxis.enabled)
            euler.x = Mathf.Clamp(xAxis.currentAngle, xAxis.minAngle, xAxis.maxAngle);
        
        if (yAxis.enabled)
            euler.y = Mathf.Clamp(yAxis.currentAngle, yAxis.minAngle, yAxis.maxAngle);
        
        if (zAxis.enabled)
            euler.z = Mathf.Clamp(zAxis.currentAngle, zAxis.minAngle, zAxis.maxAngle);
            
        return Quaternion.Euler(euler);
    }
    
    public void SetAxisAngle(JointAxis axis, float angle)
    {
        switch (axis)
        {
            case JointAxis.X:
                xAxis.currentAngle = Mathf.Clamp(angle, xAxis.minAngle, xAxis.maxAngle);
                break;
            case JointAxis.Y:
                yAxis.currentAngle = Mathf.Clamp(angle, yAxis.minAngle, yAxis.maxAngle);
                break;
            case JointAxis.Z:
                zAxis.currentAngle = Mathf.Clamp(angle, zAxis.minAngle, zAxis.maxAngle);
                break;
        }
    }
    
    public float GetAxisAngle(JointAxis axis)
    {
        switch (axis)
        {
            case JointAxis.X: return xAxis.currentAngle;
            case JointAxis.Y: return yAxis.currentAngle;
            case JointAxis.Z: return zAxis.currentAngle;
            default: return 0f;
        }
    }
}

public enum JointAxis { X, Y, Z }

[ExecuteInEditMode]
public class RobotArmManager : MonoBehaviour
{
    public enum KinematicsMode
    {
        ForwardKinematics,
        InverseKinematics
    }

    [Header("Arm Configuration")]
    [SerializeField, Range(4, 20)] private int jointCount = 6;
    [SerializeField] private float segmentLength = 1f;
    [SerializeField] private float jointRadius = 0.2f;
    [SerializeField] private float capsuleRadius = 0.1f;
    [SerializeField] private Vector3 startPosition = Vector3.zero;
    
    [Header("Target Configuration")]
    [SerializeField] private Transform target;
    [SerializeField] private Vector3 targetOffset = Vector3.zero;
    
    [Header("Joint Configurations")]
    [SerializeField] private List<JointConfig> jointConfigs = new List<JointConfig>();
    
    [Header("Materials")]
    [SerializeField] private Material jointMaterial;
    [SerializeField] private Material segmentMaterial;
    
    [Header("Kinematics Mode")]
    [SerializeField] private KinematicsMode kinematicsMode = KinematicsMode.ForwardKinematics;
    
    private List<Transform> _jointContainers = new List<Transform>();
    private List<Transform> _jointVisuals = new List<Transform>();
    private List<Transform> _segmentVisuals = new List<Transform>();
    private List<Transform> _nextJointOffsets = new List<Transform>();
    private bool _visualsInitialized = false;
    
    private bool _wasInFKMode = false;
    

    // обеспечиваем доступ к компонентам из других скриптов
    public int JointCount => jointCount;
    public float SegmentLength => segmentLength;
    public float JointRadius => jointRadius;
    public float CapsuleRadius => capsuleRadius;
    public Vector3 StartPosition => startPosition;
    public Transform Target => target;
    public Vector3 TargetOffset => targetOffset;
    public List<JointConfig> JointConfigs => jointConfigs;
    public Material JointMaterial => jointMaterial;
    public Material SegmentMaterial => segmentMaterial;
    public KinematicsMode CurrentKinematicsMode => kinematicsMode;
    
    public List<Transform> JointContainers => _jointContainers;
    public List<Transform> JointVisuals => _jointVisuals;
    public List<Transform> SegmentVisuals => _segmentVisuals;
    public List<Transform> NextJointOffsets => _nextJointOffsets;
    public bool VisualsInitialized => _visualsInitialized;

    private RobotArm robotArm;
    private RobotArmIK robotArmIK;

    // переменные фиксирующие изменения
    private int lastJointCount;
    private float lastSegmentLength;
    private float lastJointRadius;
    private float lastCapsuleRadius;
    private Vector3 lastStartPosition;
    private KinematicsMode lastKinematicsMode;
    private bool isInitialized = false;

    void OnEnable()
    {
        InitializeComponents();
        InitializeVisuals();
        CacheCurrentValues();
    }

    void Update()
    {
        if (!isInitialized)
        {
            InitializeComponents();
            InitializeVisuals();
            return;
        }
        
        // изменение режима кинематики
        bool kinematicsModeChanged = (kinematicsMode != lastKinematicsMode);
        
        // применение изменений в инспекторе
        if (HasConfigurationChanged() || kinematicsModeChanged)
        {
            UpdateArmConfiguration();
            UpdateVisuals();
            CacheCurrentValues();
        }

        UpdateKinematicsMode();
        
        UpdateTargetBehavior();
        
        _wasInFKMode = (kinematicsMode == KinematicsMode.ForwardKinematics);
    }

    private void InitializeComponents()
    {
        // если на объекте не висит скрипт прямой кинематики - добавляет
        robotArm = GetComponent<RobotArm>();
        if (robotArm == null)
        {
            robotArm = gameObject.AddComponent<RobotArm>();
        }

        // аналогично для обратной
        robotArmIK = GetComponent<RobotArmIK>();
        if (robotArmIK == null)
        {
            robotArmIK = gameObject.AddComponent<RobotArmIK>();
        }
        
        // получаем и устанавливаем все необходимые для работы пармаетры
        robotArm.robotArmManager = this;
        
        robotArmIK.robotArm = robotArm;
        robotArmIK.target = target;
        
        UpdateArmConfiguration(); // применяет параметры из инспектора
        UpdateKinematicsMode();
        isInitialized = true;
    }
    

    // поведение
    private void UpdateTargetBehavior()
    {
        if (target == null) return;
        
        if (kinematicsMode == KinematicsMode.ForwardKinematics)
        {
            // при прямой кинематике таргет следует за последним джоинтом
            UpdateTargetToFollowEndEffector();
        }
        else 
        {
            // при инверсной сбрасиваем вращения для удобства
            // свободного перемещения таргета пользователем
            if (_wasInFKMode)
            {
                target.rotation = Quaternion.identity;
            }
            
        }
    }
    
    private void UpdateTargetToFollowEndEffector()
    {
        if (_jointContainers.Count == 0) return;
        
        // последний джоинт
        Transform endEffector = GetEndEffectorTransform();
        if (endEffector == null) return;
        
        // обновление положения таргета
        target.position = endEffector.TransformPoint(targetOffset);
        target.rotation = endEffector.rotation;
    }
    
    // параметры конечного джоинта
    private Transform GetEndEffectorTransform()
    {
        if (_jointContainers.Count > 0)
        {
            return _jointContainers[_jointContainers.Count - 1];
        }
        return null;
    }
    
    private void InitializeVisuals()
    {
        ClearVisualObjects();
        CreateVisualStructure();
        UpdateVisuals();
        _visualsInitialized = true;
    }

    // обнуление вызуальной части
    private void ClearVisualObjects()
    {
        _jointContainers.Clear();
        _jointVisuals.Clear();
        _segmentVisuals.Clear();
        _nextJointOffsets.Clear();
        
        List<GameObject> childrenToDestroy = new List<GameObject>();
        foreach (Transform child in transform)
        {
            if (child != target && 
                (child.name.Contains("JointContainer") || 
                 child.name.Contains("Joint_") || 
                 child.name.Contains("Segment_") ||
                 child.name.Contains("NextJointOffset")))
            {
                childrenToDestroy.Add(child.gameObject);
            }
        }
        
        foreach (GameObject child in childrenToDestroy)
        {
            if (Application.isPlaying)
                Destroy(child);
            else
                DestroyImmediate(child);
        }
    }

    // отображение руки через джоинты(сферы) и сегменты(капсулы)
    private void CreateVisualStructure()
    {
        Transform currentParent = transform;
        
        for (int i = 0; i < jointCount; i++)
        {
            // новый контейнер джоинта и связанного с ним сегмента
            GameObject jointContainer = new GameObject($"JointContainer_{i}");
            jointContainer.transform.SetParent(currentParent, false);
            jointContainer.transform.localPosition = (i == 0) ? startPosition : Vector3.zero;
            jointContainer.transform.localRotation = Quaternion.identity;
            
            // джоинт
            GameObject jointVisual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            jointVisual.name = $"Joint_{i}";
            jointVisual.transform.SetParent(jointContainer.transform, false);
            jointVisual.transform.localPosition = Vector3.zero;
            jointVisual.transform.localRotation = Quaternion.identity;
            
            Collider collider = jointVisual.GetComponent<Collider>();
            if (collider != null)
                DestroyImmediate(collider);
            
            Renderer jointRenderer = jointVisual.GetComponent<Renderer>();
            if (jointRenderer != null && jointMaterial != null)
            {
                jointRenderer.material = jointMaterial;
            }
            
            _jointContainers.Add(jointContainer.transform);
            _jointVisuals.Add(jointVisual.transform);
            
            //связанный сегмент и хранение для следующих частей руки
            if (i < jointCount - 1)
            {
                GameObject segment = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                segment.name = $"Segment_{i}";
                segment.transform.SetParent(jointContainer.transform, false);
                segment.transform.localPosition = new Vector3(0, 0, segmentLength / 2f);
                segment.transform.localRotation = Quaternion.Euler(90f, 0, 0);
                
                Collider segmentCollider = segment.GetComponent<Collider>();
                if (segmentCollider != null)
                    DestroyImmediate(segmentCollider);
                
                Renderer segmentRenderer = segment.GetComponent<Renderer>();
                if (segmentRenderer != null && segmentMaterial != null)
                {
                    segmentRenderer.material = segmentMaterial;
                }
                
                _segmentVisuals.Add(segment.transform);
                
                GameObject nextJointOffset = new GameObject($"NextJointOffset_{i}");
                nextJointOffset.transform.SetParent(jointContainer.transform, false);
                nextJointOffset.transform.localPosition = new Vector3(0, 0, segmentLength);
                nextJointOffset.transform.localRotation = Quaternion.identity;
                
                _nextJointOffsets.Add(nextJointOffset.transform);
                currentParent = nextJointOffset.transform;
            }
        }
    }
    
    private void UpdateVisuals()
    {
        if (!_visualsInitialized) return;
        
        // позиция роборуки относительно самого первого джоинта
        if (_jointContainers.Count > 0)
        {
            _jointContainers[0].localPosition = startPosition;
        }
        
        // применение параметров джоинта
        for (int i = 0; i < _jointVisuals.Count; i++)
        {
            if (i < _jointVisuals.Count)
            {
                _jointVisuals[i].localScale = Vector3.one * jointRadius * 2f;
            }
        }
        
        // применение параметров сегмента из инспектора
        for (int i = 0; i < _segmentVisuals.Count; i++)
        {
            if (i < _segmentVisuals.Count)
            {
                _segmentVisuals[i].localPosition = new Vector3(0, 0, segmentLength / 2f);
                _segmentVisuals[i].localScale = new Vector3(
                    capsuleRadius * 2f,
                    segmentLength / 2f,
                    capsuleRadius * 2f
                );
            }
        }
        
        for (int i = 0; i < _nextJointOffsets.Count; i++)
        {
            if (i < _nextJointOffsets.Count)
            {
                _nextJointOffsets[i].localPosition = new Vector3(0, 0, segmentLength);
            }
        }
    }

    private void UpdateArmConfiguration()
    {
        // обновление парамтров прямой кинематики
        if (robotArm != null)
        {
            robotArm.robotArmManager = this;
        }

        // обновление обратной кинематики
        if (robotArmIK != null)
        {
            robotArmIK.robotArm = robotArm;
            robotArmIK.target = target;
        }

        InitializeVisuals();
    }

    private void UpdateKinematicsMode()
    {
        if (robotArm == null || robotArmIK == null) return;

        bool useIK = (kinematicsMode == KinematicsMode.InverseKinematics);

        robotArmIK.enableIK = useIK;

        if (kinematicsMode != lastKinematicsMode)
        {
            Debug.Log($"Kinematics mode changed to: {kinematicsMode}");
            lastKinematicsMode = kinematicsMode;
        }
    }

    private bool HasConfigurationChanged()
    {
        return lastJointCount != jointCount ||
               Mathf.Abs(lastSegmentLength - segmentLength) > 0.001f ||
               Mathf.Abs(lastJointRadius - jointRadius) > 0.001f ||
               Mathf.Abs(lastCapsuleRadius - capsuleRadius) > 0.001f ||
               lastStartPosition != startPosition ||
               lastKinematicsMode != kinematicsMode;
    }

    private void CacheCurrentValues()
    {
        lastJointCount = jointCount;
        lastSegmentLength = segmentLength;
        lastJointRadius = jointRadius;
        lastCapsuleRadius = capsuleRadius;
        lastStartPosition = startPosition;
        lastKinematicsMode = kinematicsMode;
    }


    public void ForceUpdate()
    {
        UpdateArmConfiguration();
        UpdateKinematicsMode();
        UpdateVisuals();
    }
    
    public void ApplyKinematicsToVisuals()
    {
        if (!_visualsInitialized || _jointContainers.Count != jointCount) return;
        
        for (int i = 0; i < jointCount; i++)
        {
            if (i < _jointContainers.Count)
            {
                _jointContainers[i].localRotation = Quaternion.identity;
            }
        }
        
        for (int i = 0; i < jointCount; i++)
        {
            if (i < jointConfigs.Count && i < _jointContainers.Count)
            {
                JointConfig config = jointConfigs[i];
                _jointContainers[i].localRotation = config.GetRotation();
            }
        }
    }
    
    
    // Editor integration
    
#if UNITY_EDITOR
    private void OnValidate()
    {
        // Ensure minimum values
        jointCount = Mathf.Max(4, jointCount);
        segmentLength = Mathf.Max(0.1f, segmentLength);
        jointRadius = Mathf.Max(0.05f, jointRadius);
        capsuleRadius = Mathf.Max(0.02f, capsuleRadius);

        // Validate joint configurations
        for (int i = 0; i < jointConfigs.Count; i++)
        {
            JointConfig config = jointConfigs[i];
            if (config != null)
            {
                config.xAxis.currentAngle = Mathf.Clamp(config.xAxis.currentAngle, config.xAxis.minAngle, config.xAxis.maxAngle);
                config.yAxis.currentAngle = Mathf.Clamp(config.yAxis.currentAngle, config.yAxis.minAngle, config.yAxis.maxAngle);
                config.zAxis.currentAngle = Mathf.Clamp(config.zAxis.currentAngle, config.zAxis.minAngle, config.zAxis.maxAngle);
            }
        }

        // Ensure joint configs array matches joint count
        while (jointConfigs.Count < jointCount)
        {
            jointConfigs.Add(new JointConfig());
        }
        while (jointConfigs.Count > jointCount)
        {
            jointConfigs.RemoveAt(jointConfigs.Count - 1);
        }

        // Auto-update in editor
        if (!Application.isPlaying)
        {
            UnityEditor.EditorApplication.delayCall += () => {
                if (this != null)
                {
                    ForceUpdate();
                }
            };
        }
    }
    
#endif
}