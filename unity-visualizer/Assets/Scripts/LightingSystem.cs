using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Script to manage lighting system in the Unity visualization
public class LightingSystem : MonoBehaviour
{
    [Header("Lighting Configuration")]
    public Light mainDirectionalLight;
    public Light[] additionalLights;
    
    [Header("Lighting Settings")]
    public bool enableShadows = true;
    public float shadowDistance = 100f;
    public float shadowResolution = 1024f;
    public Color ambientLightColor = Color.gray;
    
    [Header("Dynamic Lighting")]
    public bool enableDynamicLighting = true;
    public float lightingChangeSpeed = 0.5f;
    
    private float originalIntensity;
    private Color originalColor;
    
    // Start is called before the first frame update
    void Start()
    {
        InitializeLighting();
    }
    
    // Update is called once per frame
    void Update()
    {
        if (enableDynamicLighting)
        {
            UpdateDynamicLighting();
        }
    }
    
    // Initialize the lighting system
    private void InitializeLighting()
    {
        // Find the main directional light if not assigned
        if (mainDirectionalLight == null)
        {
            mainDirectionalLight = FindObjectOfType<Light>();
        }
        
        // Store original values for dynamic lighting
        if (mainDirectionalLight != null)
        {
            originalIntensity = mainDirectionalLight.intensity;
            originalColor = mainDirectionalLight.color;
        }
        
        // Configure shadows
        ConfigureShadows();
        
        // Set ambient light
        RenderSettings.ambientLight = ambientLightColor;
    }
    
    // Configure shadow settings
    private void ConfigureShadows()
    {
        if (mainDirectionalLight != null)
        {
            if (enableShadows)
            {
                mainDirectionalLight.shadows = LightShadows.Soft;
                QualitySettings.shadowDistance = shadowDistance;
                QualitySettings.shadowResolution = (ShadowResolution)Mathf.RoundToInt(shadowResolution / 256f);
            }
            else
            {
                mainDirectionalLight.shadows = LightShadows.None;
            }
        }
        
        // Apply shadow settings to additional lights
        if (additionalLights != null)
        {
            foreach (Light light in additionalLights)
            {
                if (enableShadows)
                {
                    light.shadows = LightShadows.Soft;
                }
                else
                {
                    light.shadows = LightShadows.None;
                }
            }
        }
    }
    
    // Update dynamic lighting effects
    private void UpdateDynamicLighting()
    {
        if (mainDirectionalLight != null)
        {
            // Simulate dynamic lighting changes (e.g., time of day)
            float time = Time.time * lightingChangeSpeed;
            float intensityVariation = Mathf.Sin(time) * 0.1f;
            float colorVariation = Mathf.Cos(time * 0.5f) * 0.05f;
            
            mainDirectionalLight.intensity = originalIntensity + intensityVariation;
            mainDirectionalLight.color = originalColor + new Color(colorVariation, colorVariation, colorVariation);
        }
    }
    
    // Update lighting based on external data
    public void UpdateLightingFromExternalData(float intensity, Color color, bool shadowsEnabled)
    {
        if (mainDirectionalLight != null)
        {
            mainDirectionalLight.intensity = intensity;
            mainDirectionalLight.color = color;
        }
        
        enableShadows = shadowsEnabled;
        ConfigureShadows();
    }
    
    // Toggle lighting quality
    public void SetLightingQuality(string quality)
    {
        switch (quality)
        {
            case "low":
                QualitySettings.shadowResolution = ShadowResolution.Low;
                QualitySettings.shadowDistance = 50f;
                break;
            case "medium":
                QualitySettings.shadowResolution = ShadowResolution.Medium;
                QualitySettings.shadowDistance = 100f;
                break;
            case "high":
                QualitySettings.shadowResolution = ShadowResolution.High;
                QualitySettings.shadowDistance = 150f;
                break;
        }
    }
}