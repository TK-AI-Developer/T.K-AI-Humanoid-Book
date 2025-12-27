using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Script to manage the humanoid model visualization
public class HumanoidModelVisualizer : MonoBehaviour
{
    [Header("Model Configuration")]
    public GameObject torso;
    public GameObject head;
    public GameObject leftArm;
    public GameObject rightArm;
    public GameObject leftLeg;
    public GameObject rightLeg;
    
    [Header("Sensor Visualizations")]
    public GameObject lidarSensor;
    public GameObject cameraSensor;
    public GameObject imuSensor;
    
    [Header("Visualization Settings")]
    public bool showSensors = true;
    public bool animateModel = true;
    
    // Animation parameters
    private float animationSpeed = 1.0f;
    private float armSwingAmount = 0.3f;
    private float legSwingAmount = 0.4f;
    
    // Start is called before the first frame update
    void Start()
    {
        InitializeModel();
    }
    
    // Update is called once per frame
    void Update()
    {
        if (animateModel)
        {
            AnimateModel();
        }
        
        UpdateSensorVisualizations();
    }
    
    // Initialize the model components
    private void InitializeModel()
    {
        // Verify all required model components are assigned
        if (torso == null) torso = transform.Find("Torso")?.gameObject;
        if (head == null) head = transform.Find("Head")?.gameObject;
        if (leftArm == null) leftArm = transform.Find("LeftArm")?.gameObject;
        if (rightArm == null) rightArm = transform.Find("RightArm")?.gameObject;
        if (leftLeg == null) leftLeg = transform.Find("LeftLeg")?.gameObject;
        if (rightLeg == null) rightLeg = transform.Find("RightLeg")?.gameObject;
        
        // Initialize sensor visualizations
        InitializeSensors();
    }
    
    // Initialize sensor visualizations
    private void InitializeSensors()
    {
        if (lidarSensor == null) lidarSensor = transform.Find("LiDAR_Sensor")?.gameObject;
        if (cameraSensor == null) cameraSensor = transform.Find("Camera_Sensor")?.gameObject;
        if (imuSensor == null) imuSensor = transform.Find("IMU_Sensor")?.gameObject;
    }
    
    // Animate the model
    private void AnimateModel()
    {
        // Simple walking animation
        float time = Time.time * animationSpeed;
        
        // Swing arms
        if (leftArm != null)
        {
            leftArm.transform.localRotation = Quaternion.Euler(
                0, 
                0, 
                Mathf.Sin(time) * armSwingAmount
            );
        }
        
        if (rightArm != null)
        {
            rightArm.transform.localRotation = Quaternion.Euler(
                0, 
                0, 
                Mathf.Cos(time) * armSwingAmount
            );
        }
        
        // Swing legs
        if (leftLeg != null)
        {
            leftLeg.transform.localRotation = Quaternion.Euler(
                0, 
                0, 
                Mathf.Cos(time) * legSwingAmount
            );
        }
        
        if (rightLeg != null)
        {
            rightLeg.transform.localRotation = Quaternion.Euler(
                0, 
                0, 
                Mathf.Sin(time) * legSwingAmount
            );
        }
    }
    
    // Update sensor visualizations
    private void UpdateSensorVisualizations()
    {
        if (lidarSensor != null) lidarSensor.SetActive(showSensors);
        if (cameraSensor != null) cameraSensor.SetActive(showSensors);
        if (imuSensor != null) imuSensor.SetActive(showSensors);
    }
    
    // Update model pose based on external data
    public void UpdateModelPose(Vector3 position, Quaternion rotation)
    {
        transform.position = position;
        transform.rotation = rotation;
    }
    
    // Update joint angles based on external data
    public void UpdateJointAngles(Dictionary<string, float> jointAngles)
    {
        if (jointAngles.ContainsKey("left_arm_joint"))
        {
            if (leftArm != null)
            {
                leftArm.transform.localRotation = Quaternion.Euler(0, 0, jointAngles["left_arm_joint"]);
            }
        }
        
        if (jointAngles.ContainsKey("right_arm_joint"))
        {
            if (rightArm != null)
            {
                rightArm.transform.localRotation = Quaternion.Euler(0, 0, jointAngles["right_arm_joint"]);
            }
        }
        
        if (jointAngles.ContainsKey("left_leg_joint"))
        {
            if (leftLeg != null)
            {
                leftLeg.transform.localRotation = Quaternion.Euler(0, 0, jointAngles["left_leg_joint"]);
            }
        }
        
        if (jointAngles.ContainsKey("right_leg_joint"))
        {
            if (rightLeg != null)
            {
                rightLeg.transform.localRotation = Quaternion.Euler(0, 0, jointAngles["right_leg_joint"]);
            }
        }
    }
}