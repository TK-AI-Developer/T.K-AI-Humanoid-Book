using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Script to manage animation system for the humanoid model
public class AnimationSystem : MonoBehaviour
{
    [Header("Animation Configuration")]
    public HumanoidModelVisualizer modelVisualizer;
    public Animator animator;
    
    [Header("Animation Parameters")]
    public bool enableAnimations = true;
    public float animationSpeed = 1.0f;
    public AnimationCurve animationSmoothing = AnimationCurve.EaseInOut(0, 0, 1, 1);
    
    [Header("Predefined Animations")]
    public string[] availableAnimations;
    public string currentAnimation = "idle";
    
    [Header("Animation Settings")]
    public float transitionSpeed = 2.0f;
    public bool loopAnimation = true;
    
    // Animation states
    private Dictionary<string, AnimationState> animationStates = new Dictionary<string, AnimationState>();
    private string targetAnimation = "idle";
    private float animationTransition = 0f;
    
    // Start is called before the first frame update
    void Start()
    {
        InitializeAnimationSystem();
    }
    
    // Update is called once per frame
    void Update()
    {
        if (enableAnimations)
        {
            UpdateAnimationTransitions();
        }
    }
    
    // Initialize the animation system
    private void InitializeAnimationSystem()
    {
        // Find the model visualizer if not assigned
        if (modelVisualizer == null)
        {
            modelVisualizer = GetComponent<HumanoidModelVisualizer>();
        }
        
        // Get the animator component
        if (animator == null)
        {
            animator = GetComponent<Animator>();
        }
        
        // Register available animations
        availableAnimations = new string[] { "idle", "walk", "run", "jump", "wave", "dance" };
        
        // Initialize animation states
        foreach (string animName in availableAnimations)
        {
            animationStates[animName] = new AnimationState(animName, false);
        }
        
        // Set initial animation
        PlayAnimation("idle");
    }
    
    // Play a specific animation
    public void PlayAnimation(string animationName)
    {
        if (animationStates.ContainsKey(animationName))
        {
            targetAnimation = animationName;
            animationTransition = 0f;
        }
        else
        {
            Debug.LogWarning("Animation '" + animationName + "' not found in available animations.");
        }
    }
    
    // Update animation transitions
    private void UpdateAnimationTransitions()
    {
        if (animationTransition < 1.0f)
        {
            animationTransition += Time.deltaTime * transitionSpeed;
            animationTransition = Mathf.Clamp01(animationTransition);
            
            // Apply transition effect if needed
            ApplyAnimationTransition();
        }
    }
    
    // Apply animation transition effect
    private void ApplyAnimationTransition()
    {
        // This would handle smooth transitions between animations
        // In a real implementation, this would blend between animation states
    }
    
    // Update animation based on external data
    public void UpdateAnimationFromExternalData(string animationName, float speed, bool loop)
    {
        PlayAnimation(animationName);
        animationSpeed = speed;
        loopAnimation = loop;
    }
    
    // Set animation speed
    public void SetAnimationSpeed(float speed)
    {
        animationSpeed = speed;
        if (animator != null)
        {
            animator.speed = speed;
        }
    }
    
    // Get current animation state
    public string GetCurrentAnimation()
    {
        return currentAnimation;
    }
    
    // Check if animation is playing
    public bool IsAnimationPlaying()
    {
        return animationTransition >= 1.0f;
    }
    
    // Animation state data structure
    [System.Serializable]
    public class AnimationState
    {
        public string name;
        public bool isPlaying;
        public float progress;
        public float speed;
        
        public AnimationState(string n, bool playing)
        {
            name = n;
            isPlaying = playing;
            progress = 0f;
            speed = 1.0f;
        }
    }
}