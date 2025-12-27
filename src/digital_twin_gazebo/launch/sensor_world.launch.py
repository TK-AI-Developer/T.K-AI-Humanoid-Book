<?xml version="1.0"?>
<launch>
  <!-- Arguments -->
  <arg name="world" default="simple_world.sdf" description="Choose one of the world files from `/digital_twin_gazebo/worlds`"/>
  <arg name="paused" default="false" description="Starts Gazebo in paused mode"/>
  <arg name="use_sim_time" default="true" description="Sets the use_sim_time parameter for all nodes launched in the system"/>
  <arg name="gui" default="true" description="Launch the gazebo GUI"/>
  <arg name="headless" default="false" description="Enable headless mode (no GUI)"/>
  <arg name="debug" default="false" description="Starts gzserver in debug mode using gdb"/>
  <arg name="physics" default="ode" description="Physics engine to use: ode, dart, bullet, simbody"/>

  <!-- Set use_sim_time parameter -->
  <param name="use_sim_time" value="$(var use_sim_time)"/>

  <!-- Launch Gazebo with the specified world -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gzserver.launch.py">
    <arg name="world" value="$(find-pkg-share digital_twin_gazebo)/worlds/$(var world)"/>
    <arg name="paused" value="$(var paused)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
    <arg name="headless" value="$(var headless)"/>
    <arg name="debug" value="$(var debug)"/>
    <arg name="physics" value="$(var physics)"/>
  </include>

  <!-- Launch Gazebo client GUI -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gzclient.launch.py" if="$(var gui)"/>

  <!-- Robot State Publisher for the enhanced humanoid model with sensors -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="robot_description" value="$(command 'xacro $(find-pkg-share digital_twin_gazebo)/models/enhanced_humanoid.urdf')"/>
  </node>

  <!-- Spawn the enhanced humanoid model in Gazebo -->
  <node pkg="gazebo_ros" exec="spawn_entity.py" 
        args="-entity enhanced_humanoid -topic robot_description -x 0 -y 0 -z 1.0">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
</launch>