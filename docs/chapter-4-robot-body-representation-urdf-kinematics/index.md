---
title: Chapter 4 - Robot Body Representation (URDF & Kinematics)
sidebar_position: 4
---

# Robot Body Representation (URDF & Kinematics)

## Learning Objectives

After completing this chapter, readers will be able to:
1. Create and understand URDF (Unified Robot Description Format) files for robot representation
2. Define robot kinematic chains and joint relationships
3. Distinguish between collision and visual models in robot representation
4. Apply forward and inverse kinematics for robot motion planning
5. Validate and debug robot models for simulation and real-world deployment

## URDF and SDF Formats

### Unified Robot Description Format (URDF)

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS. It defines the physical and visual properties of a robot, including links, joints, and their relationships.

#### URDF Structure

A URDF file consists of several main elements:

- **Links**: Rigid bodies that make up the robot
- **Joints**: Connections between links with specific degrees of freedom
- **Materials**: Visual properties like color and texture
- **Gazebo Extensions**: Simulation-specific properties

#### Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### URDF Elements in Detail

#### Links

Links represent rigid bodies in the robot. Each link can have:

- **Visual**: How the link appears in visualizations
- **Collision**: How the link interacts in collision detection
- **Inertial**: Mass, center of mass, and inertia properties

```xml
<link name="link_name">
  <visual>
    <geometry>
      <!-- Shape: box, cylinder, sphere, mesh -->
      <box size="0.1 0.2 0.3"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <material name="material_name"/>
  </visual>

  <collision>
    <!-- Similar to visual but for collision detection -->
    <geometry>
      <cylinder length="0.5" radius="0.1"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </collision>

  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

#### Joints

Joints define the connection between links and specify how they can move relative to each other:

- **Revolute**: Rotational joint with one degree of freedom
- **Prismatic**: Linear joint with one degree of freedom
- **Continuous**: Rotational joint without limits
- **Fixed**: No movement between links
- **Floating**: Six degrees of freedom
- **Planar**: Movement in a plane

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### URDF Best Practices

#### File Organization
- Use XACRO for complex robots to reduce redundancy
- Separate files for different robot components
- Use relative paths for mesh files

#### XACRO Example

XACRO (XML Macros) extends URDF with features like variables, math, and macros:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />

  <!-- Macro for creating wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <link name="base">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <xacro:wheel prefix="front_left" parent="base" xyz="0.15 0.15 0" rpy="0 0 0"/>
  <xacro:wheel prefix="front_right" parent="base" xyz="0.15 -0.15 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_left" parent="base" xyz="-0.15 0.15 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_right" parent="base" xyz="-0.15 -0.15 0" rpy="0 0 0"/>

</robot>
```

### SDF (Simulation Description Format)

SDF is used primarily by Gazebo for simulation. While URDF is more common for ROS, SDF offers additional features for simulation:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_model">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.4</iyy>
          <iyz>0.0</iyz>
          <izz>0.4</izz>
        </inertia>
      </inertial>

      <visual name="chassis_visual">
        <geometry>
          <box>
            <size>1.0 0.5 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>

      <collision name="chassis_collision">
        <geometry>
          <box>
            <size>1.0 0.5 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="joint1" type="revolute">
      <parent>chassis</parent>
      <child>arm</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

## Links, Joints, and Coordinate Frames

### Robot Coordinate Systems

In robotics, coordinate frames are essential for defining positions and orientations:

#### World Coordinate Frame
- Fixed reference frame for the environment
- Origin typically at a convenient location in the workspace

#### Base Coordinate Frame
- Attached to the robot's base
- Origin typically at the robot's center or a specific reference point

#### Link Coordinate Frames
- Each link has its own coordinate frame
- Defined relative to its parent link

#### End-Effector Frame
- Attached to the robot's end effector
- Used for task specification and control

### Joint Types and Degrees of Freedom

#### Revolute Joint
- One rotational degree of freedom
- Defined by axis of rotation and joint limits
- Common in robot arms and legs

#### Prismatic Joint
- One translational degree of freedom
- Linear motion along specified axis
- Used in telescoping mechanisms

#### Spherical Joint
- Three rotational degrees of freedom
- Allows arbitrary rotation
- Complex to implement but provides high mobility

#### Planar Joint
- Motion constrained to a plane
- Two translational DOF + one rotational DOF
- Useful for mobile bases

### TF (Transform) Tree

The TF tree represents the relationships between all coordinate frames in the robot:

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class RobotTfPublisher(Node):
    def __init__(self):
        super().__init__('robot_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to broadcast transforms
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

    def broadcast_transforms(self):
        """
        Broadcast the robot's transform tree
        """
        # Base to arm transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'arm_link'

        # Set translation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.3

        # Set rotation (identity in this example)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
```

## Humanoid Kinematic Chains

### Humanoid Robot Anatomy

Humanoid robots typically have kinematic chains that mimic human body structure:

#### Lower Body Kinematic Chain
- **Hip Joint**: Complex joint with multiple DOF
- **Thigh**: Link connecting hip to knee
- **Knee Joint**: Single DOF for flexion/extension
- **Shin**: Link connecting knee to ankle
- **Ankle Joint**: Multiple DOF for foot orientation

#### Upper Body Kinematic Chain
- **Shoulder Complex**: Multiple joints for arm positioning
- **Upper Arm**: Link from shoulder to elbow
- **Elbow Joint**: Single DOF for flexion/extension
- **Forearm**: Link from elbow to wrist
- **Wrist Joint**: Multiple DOF for hand orientation

### Kinematic Chain Representation

```python
class KinematicChain:
    def __init__(self, name, base_link, tip_link):
        self.name = name
        self.base_link = base_link
        self.tip_link = tip_link
        self.joints = []
        self.links = []

    def add_joint(self, joint_name, joint_type, parent_link, child_link, axis, limits):
        """
        Add a joint to the kinematic chain
        """
        joint = {
            'name': joint_name,
            'type': joint_type,
            'parent': parent_link,
            'child': child_link,
            'axis': axis,  # [x, y, z] axis of rotation/translation
            'limits': limits  # [lower, upper] for revolute joints
        }
        self.joints.append(joint)

    def get_chain_info(self):
        """
        Get information about the kinematic chain
        """
        return {
            'name': self.name,
            'base_link': self.base_link,
            'tip_link': self.tip_link,
            'num_joints': len(self.joints),
            'joint_names': [j['name'] for j in self.joints],
            'joint_types': [j['type'] for j in self.joints]
        }

class HumanoidRobot:
    def __init__(self):
        # Create kinematic chains for humanoid robot
        self.right_arm = KinematicChain('right_arm', 'torso', 'right_hand')
        self.left_arm = KinematicChain('left_arm', 'torso', 'left_hand')
        self.right_leg = KinematicChain('right_leg', 'pelvis', 'right_foot')
        self.left_leg = KinematicChain('left_leg', 'pelvis', 'left_foot')
        self.head = KinematicChain('head', 'torso', 'head_link')

        # Define joints for right arm (simplified)
        self.right_arm.add_joint(
            'right_shoulder_pan', 'revolute', 'torso', 'right_upper_arm',
            [0, 0, 1], [-1.57, 1.57]
        )
        self.right_arm.add_joint(
            'right_shoulder_lift', 'revolute', 'right_upper_arm', 'right_lower_arm',
            [0, 1, 0], [-1.57, 1.57]
        )
        self.right_arm.add_joint(
            'right_elbow_flex', 'revolute', 'right_lower_arm', 'right_hand',
            [0, 0, 1], [-2.0, 0.5]
        )

    def get_kinematic_chain(self, chain_name):
        """
        Get a specific kinematic chain
        """
        chains = {
            'right_arm': self.right_arm,
            'left_arm': self.left_arm,
            'right_leg': self.right_leg,
            'left_leg': self.left_leg,
            'head': self.head
        }
        return chains.get(chain_name, None)
```

### Forward Kinematics

Forward kinematics computes the end-effector position and orientation given joint angles:

```python
import numpy as np
from math import sin, cos

def dh_transform(a, alpha, d, theta):
    """
    Denavit-Hartenberg transformation matrix
    """
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

class ForwardKinematics:
    def __init__(self, dh_params):
        """
        Initialize with DH parameters
        dh_params: list of [a, alpha, d, theta] for each joint
        """
        self.dh_params = dh_params

    def calculate_pose(self, joint_angles):
        """
        Calculate end-effector pose given joint angles
        """
        if len(joint_angles) != len(self.dh_params):
            raise ValueError("Number of joint angles must match DH parameters")

        # Start with identity matrix
        T = np.eye(4)

        for i, (a, alpha, d, _) in enumerate(self.dh_params):
            # Update theta with actual joint angle
            theta = joint_angles[i]
            T_joint = dh_transform(a, alpha, d, theta)
            T = T @ T_joint

        return T  # Homogeneous transformation matrix

# Example: 2-DOF planar manipulator
dh_params_2dof = [
    [0.5, 0, 0, 0],  # First joint
    [0.5, 0, 0, 0]   # Second joint
]

fk = ForwardKinematics(dh_params_2dof)
joint_angles = [0.5, 0.3]  # radians
end_pose = fk.calculate_pose(joint_angles)
print(f"End-effector pose:\n{end_pose}")
```

### Inverse Kinematics

Inverse kinematics computes joint angles needed to achieve a desired end-effector pose:

```python
class InverseKinematics:
    def __init__(self, dh_params):
        self.dh_params = dh_params

    def solve_2dof_planar(self, x, y):
        """
        Solve inverse kinematics for 2-DOF planar manipulator
        """
        # Link lengths
        l1 = self.dh_params[0][0]  # a1
        l2 = self.dh_params[1][0]  # a2

        # Check if position is reachable
        distance = np.sqrt(x**2 + y**2)
        if distance > l1 + l2:
            raise ValueError("Position is out of reach")

        if distance < abs(l1 - l2):
            raise ValueError("Position is inside workspace but unreachable due to joint limits")

        # Calculate joint angles using law of cosines
        cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        sin_theta2 = np.sqrt(1 - cos_theta2**2)

        theta2 = np.arctan2(sin_theta2, cos_theta2)

        k1 = l1 + l2 * cos_theta2
        k2 = l2 * sin_theta2

        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return [theta1, theta2]

    def jacobian_ik(self, target_pose, current_joints, steps=100, alpha=0.01):
        """
        Solve inverse kinematics using Jacobian transpose method
        """
        joints = np.array(current_joints)

        for _ in range(steps):
            # Calculate current end-effector position
            current_pose = self.forward_kinematics(joints)
            current_pos = current_pose[:3, 3]
            target_pos = target_pose[:3, 3]

            # Calculate error
            error = target_pos - current_pos

            if np.linalg.norm(error) < 0.001:  # Convergence threshold
                break

            # Calculate Jacobian
            J = self.calculate_jacobian(joints)

            # Update joint angles
            joints += alpha * J.T @ error

        return joints.tolist()

    def calculate_jacobian(self, joints):
        """
        Calculate geometric Jacobian for the robot
        """
        # This is a simplified version - in practice, you'd calculate
        # the full Jacobian based on the robot's kinematic structure
        # For demonstration purposes:
        pass

    def forward_kinematics(self, joints):
        """
        Forward kinematics to get current end-effector pose
        """
        fk = ForwardKinematics(self.dh_params)
        return fk.calculate_pose(joints)
```

## Collision vs Visual Models

### Visual Models

Visual models define how the robot appears in simulation and visualization:

#### Properties
- **Geometry**: Shape definition (box, cylinder, sphere, mesh)
- **Materials**: Color, texture, and visual properties
- **Resolution**: Level of detail for visualization

#### Example Visual Definition

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://robot_description/meshes/link_visual.dae" scale="1 1 1"/>
  </geometry>
  <material name="light_grey">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>
</visual>
```

### Collision Models

Collision models define how the robot interacts with the environment in collision detection:

#### Properties
- **Geometry**: Shape definition for collision detection
- **Complexity**: Balance between accuracy and performance
- **Safety**: Must fully encompass the physical robot

#### Example Collision Definition

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://robot_description/meshes/link_collision.stl" scale="1 1 1"/>
  </geometry>
</collision>
```

### Differences and Best Practices

#### Visual vs Collision Complexity
- **Visual**: Can be highly detailed with textures
- **Collision**: Should be simplified for performance
- **Balance**: Detailed enough for safety, simple enough for performance

#### Safety Considerations
- Collision models should be conservative (slightly larger than visual models)
- Ensure no part of the physical robot extends beyond collision geometry
- Consider tolerances and manufacturing variations

#### Performance Optimization
- Use primitive shapes (boxes, cylinders) where possible for collision
- Simplify mesh models for collision while keeping detailed visual models
- Consider multiple collision models at different levels of detail

### Validation and Debugging

#### URDF Validation

```python
import xml.etree.ElementTree as ET
from collections import defaultdict

def validate_urdf(urdf_file):
    """
    Basic URDF validation
    """
    try:
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        # Check for required elements
        if root.tag != 'robot':
            raise ValueError("URDF must have 'robot' as root element")

        robot_name = root.get('name')
        if not robot_name:
            raise ValueError("Robot must have a name attribute")

        # Validate structure
        links = root.findall('link')
        joints = root.findall('joint')

        if not links:
            raise ValueError("Robot must have at least one link")

        # Check joint-link connections
        link_names = {link.get('name') for link in links}
        for joint in joints:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')

            if parent not in link_names:
                raise ValueError(f"Joint {joint.get('name')} references non-existent parent link: {parent}")
            if child not in link_names:
                raise ValueError(f"Joint {joint.get('name')} references non-existent child link: {child}")

        print(f"URDF validation passed for robot: {robot_name}")
        print(f"Found {len(links)} links and {len(joints)} joints")

        return True

    except ET.ParseError as e:
        print(f"XML parsing error: {e}")
        return False
    except Exception as e:
        print(f"URDF validation error: {e}")
        return False

def check_kinematic_chain(urdf_file):
    """
    Check if the URDF has a proper kinematic chain
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # Build joint graph
    joint_graph = defaultdict(list)
    joint_info = {}

    for joint in root.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        joint_type = joint.get('type')

        joint_graph[parent].append(child)
        joint_info[(parent, child)] = joint_type

    # Find base link (link that is a child but not a parent)
    all_children = set()
    all_parents = set()

    for joint in root.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')

        all_parents.add(parent)
        all_children.add(child)

    base_links = all_parents - all_children

    if not base_links:
        print("Error: No base link found (cyclic joint structure)")
        return False

    print(f"Base links found: {base_links}")

    # Check for proper tree structure (no loops)
    visited = set()
    queue = list(base_links)

    while queue:
        current = queue.pop(0)
        if current in visited:
            print("Error: Cyclic joint structure detected")
            return False
        visited.add(current)

        for child in joint_graph[current]:
            queue.append(child)

    print("Kinematic chain validation passed")
    return True
```

## Key Claims Requiring Citations

1. URDF provides a standardized XML-based format for describing robot geometry and kinematics (Citation needed - see references.md)

2. Kinematic chains in humanoid robots mimic human body structure with multiple degrees of freedom (Citation needed - see references.md)

3. Forward kinematics computes end-effector pose from joint angles using transformation matrices (Citation needed - see references.md)

4. Inverse kinematics computes joint angles needed to achieve desired end-effector poses (Citation needed - see references.md)

5. Collision models must conservatively encompass the physical robot for safe operation (Citation needed - see references.md)

6. Visual and collision models serve different purposes and may have different levels of detail (Citation needed - see references.md)

## Reproducibility Notes

- All examples assume ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS
- Required packages: xacro, robot_state_publisher, joint_state_publisher
- Mesh files should be in standard formats (STL, DAE, OBJ)
- Validation tools require Python XML processing libraries

## Summary

This chapter covered robot body representation using URDF and SDF formats, the definition of links and joints in kinematic chains, and the specific considerations for humanoid robot kinematics. We explored the differences between collision and visual models, and discussed validation techniques for robot descriptions. Proper robot representation is fundamental to both simulation and real-world robot control. The next chapter will explore digital twins and physics simulation environments.

---

## References

For full citations, see [References](/docs/references.md).