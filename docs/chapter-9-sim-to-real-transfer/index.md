---
title: Chapter 9 - Sim-to-Real Transfer
sidebar_position: 9
---

# Sim-to-Real Transfer

## Learning Objectives

After completing this chapter, readers will be able to:
1. Identify and analyze the fundamental challenges in transferring policies from simulation to reality
2. Apply domain randomization and system identification techniques to improve transfer
3. Evaluate and mitigate latency and sensor mismatch issues
4. Deploy robotic policies on Jetson edge devices with appropriate adaptations
5. Recognize and handle common failure modes in sim-to-real transfer

## Introduction to the Reality Gap

The sim-to-real transfer problem represents one of the most significant challenges in robotics research. While simulation provides a safe, controllable, and cost-effective environment for training robotic policies, the gap between simulated and real-world conditions often prevents successful deployment of learned behaviors.

### The Fundamental Challenge

The reality gap encompasses all discrepancies between simulation and the real world:

- **Dynamics Mismatch**: Differences in robot dynamics, friction, and actuator behavior
- **Sensor Noise**: Real sensors exhibit noise patterns not present in simulation
- **Environmental Factors**: Lighting, surface properties, and external disturbances
- **Latency Differences**: Communication delays and processing times vary between simulation and reality
- **Modeling Inaccuracies**: Simplified models in simulation fail to capture complex real-world phenomena

### Taxonomy of Reality Gaps

#### Physical Reality Gaps
- **Inertial Properties**: Mass, center of mass, and moment of inertia differences
- **Friction Models**: Static, dynamic, and viscous friction variations
- **Actuator Dynamics**: Response time, torque limits, and non-linearities
- **Flexibility**: Structural compliance not modeled in simulation

#### Perceptual Reality Gaps
- **Visual Perception**: Lighting, textures, and sensor noise differences
- **Depth Perception**: Depth sensor noise and accuracy variations
- **Proprioceptive Sensing**: Joint encoder accuracy and drift
- **Tactile Sensing**: Contact detection and force measurement differences

#### Temporal Reality Gaps
- **Processing Latency**: Computation time affecting control frequency
- **Communication Latency**: Network delays in distributed systems
- **Sensor-Actuator Synchronization**: Timing mismatches between perception and action

## Domain Randomization Techniques

Domain randomization is a powerful approach to bridge the sim-to-real gap by training policies across a wide range of randomized environmental conditions.

### Basic Domain Randomization

```python
import numpy as np
import random

class DomainRandomization:
    def __init__(self, env):
        self.env = env
        self.param_ranges = {
            'mass_multiplier': (0.5, 1.5),
            'friction': (0.05, 1.0),
            'restitution': (0.0, 0.5),
            'gravity': (-12.0, -8.0),
            'lighting': (0.3, 2.5),
            'texture_variation': (0.0, 1.0),
            'sensor_noise_std': (0.0, 0.1)
        }

    def randomize_environment(self):
        """
        Randomize environment parameters at the beginning of each episode
        """
        for param, (min_val, max_val) in self.param_ranges.items():
            if param == 'mass_multiplier':
                self._randomize_masses(min_val, max_val)
            elif param == 'friction':
                self._randomize_friction(min_val, max_val)
            elif param == 'restitution':
                self._randomize_restitution(min_val, max_val)
            elif param == 'gravity':
                self._randomize_gravity(min_val, max_val)
            elif param == 'lighting':
                self._randomize_lighting(min_val, max_val)
            elif param == 'texture_variation':
                self._randomize_textures(min_val, max_val)
            elif param == 'sensor_noise_std':
                self._randomize_sensor_noise(min_val, max_val)

    def _randomize_masses(self, min_mult, max_mult):
        """
        Randomize robot and object masses
        """
        for link in self.env.robot_links:
            base_mass = self.env.get_base_mass(link)
            randomized_mass = base_mass * np.random.uniform(min_mult, max_mult)
            self.env.set_link_mass(link, randomized_mass)

    def _randomize_friction(self, min_friction, max_friction):
        """
        Randomize friction coefficients
        """
        for link in self.env.robot_links:
            friction = np.random.uniform(min_friction, max_friction)
            self.env.set_link_friction(link, friction)

    def _randomize_sensor_noise(self, min_noise, max_noise):
        """
        Randomize sensor noise characteristics
        """
        noise_std = np.random.uniform(min_noise, max_noise)
        self.env.set_sensor_noise_std(noise_std)
```

### Adaptive Domain Randomization

Adaptive domain randomization adjusts the randomization distribution based on policy performance:

```python
class AdaptiveDomainRandomization:
    def __init__(self, base_randomizer, performance_threshold=0.7):
        self.base_randomizer = base_randomizer
        self.performance_threshold = performance_threshold
        self.performance_history = []
        self.param_distributions = {}  # Track distribution parameters

    def update_randomization(self, episode_performance):
        """
        Update randomization distributions based on performance
        """
        self.performance_history.append(episode_performance)

        # If performance is too high, increase randomization range
        if episode_performance > self.performance_threshold:
            self._increase_randomization_range()
        else:
            # If performance is too low, decrease range to help learning
            self._decrease_randomization_range()

    def _increase_randomization_range(self):
        """
        Increase the range of randomization parameters
        """
        for param in self.param_distributions:
            if 'range' in self.param_distributions[param]:
                current_range = self.param_distributions[param]['range']
                new_range = (current_range[0] * 0.9, current_range[1] * 1.1)
                self.param_distributions[param]['range'] = new_range

    def _decrease_randomization_range(self):
        """
        Decrease the range of randomization parameters
        """
        for param in self.param_distributions:
            if 'range' in self.param_distributions[param]:
                current_range = self.param_distributions[param]['range']
                new_range = (current_range[0] * 1.05, current_range[1] * 0.95)
                # Ensure range doesn't become too small
                new_range = (max(0.1, new_range[0]), min(2.0, new_range[1]))
                self.param_distributions[param]['range'] = new_range

    def randomize_environment(self):
        """
        Apply adaptive randomization based on updated distributions
        """
        for param, dist_params in self.param_distributions.items():
            if 'range' in dist_params:
                min_val, max_val = dist_params['range']
                randomized_val = np.random.uniform(min_val, max_val)
                self._apply_parameter(param, randomized_val)

    def _apply_parameter(self, param, value):
        """
        Apply the randomized parameter value to the environment
        """
        if param == 'mass_multiplier':
            self._randomize_masses(value, value)  # Fixed value instead of range
        elif param == 'friction':
            self._randomize_friction(value, value)
        # Add other parameter applications
```

### Systematic Domain Randomization

Systematic domain randomization uses structured approaches to cover the parameter space:

```python
class SystematicDomainRandomization:
    def __init__(self, env, n_bins=5):
        self.env = env
        self.n_bins = n_bins
        self.param_grid = self._create_parameter_grid()
        self.current_idx = 0

    def _create_parameter_grid(self):
        """
        Create a grid of parameter combinations to systematically explore
        """
        mass_multipliers = np.linspace(0.5, 1.5, self.n_bins)
        frictions = np.linspace(0.05, 1.0, self.n_bins)
        lightings = np.linspace(0.3, 2.5, self.n_bins)

        # Create all combinations
        param_grid = []
        for mass_mult in mass_multipliers:
            for friction in frictions:
                for lighting in lightings:
                    param_grid.append({
                        'mass_multiplier': mass_mult,
                        'friction': friction,
                        'lighting': lighting
                    })

        return param_grid

    def randomize_environment(self):
        """
        Cycle through parameter combinations in a systematic way
        """
        if self.current_idx >= len(self.param_grid):
            self.current_idx = 0  # Cycle back to beginning

        params = self.param_grid[self.current_idx]
        self._apply_parameters(params)
        self.current_idx += 1

    def _apply_parameters(self, params):
        """
        Apply the current parameter combination
        """
        self._randomize_masses(params['mass_multiplier'], params['mass_multiplier'])
        self._randomize_friction(params['friction'], params['friction'])
        self._randomize_lighting(params['lighting'], params['lighting'])
```

## Latency and Sensor Mismatch

### Communication Latency

Communication delays between sensors, controllers, and actuators can significantly impact robotic performance:

```python
class LatencyCompensator:
    def __init__(self, max_latency=0.1):  # 100ms max latency
        self.max_latency = max_latency
        self.state_buffer = []  # Store past states
        self.action_buffer = []  # Store past actions
        self.latency_estimate = 0.0

    def add_state(self, state, timestamp):
        """
        Add state with timestamp to buffer for latency compensation
        """
        self.state_buffer.append((state, timestamp))
        # Keep only recent states within max latency window
        current_time = time.time()
        self.state_buffer = [
            (s, t) for s, t in self.state_buffer
            if current_time - t <= self.max_latency
        ]

    def predict_current_state(self, current_time):
        """
        Predict current state based on delayed observations
        """
        if not self.state_buffer:
            return None

        # Use the most recent state and apply motion model
        recent_state, recent_time = self.state_buffer[-1]
        time_diff = current_time - recent_time

        # Apply simple motion model (can be more sophisticated)
        predicted_state = self._apply_motion_model(recent_state, time_diff)
        return predicted_state

    def _apply_motion_model(self, state, dt):
        """
        Apply motion model to predict state after time dt
        """
        # Example: integrate velocities to predict positions
        predicted_state = state.copy()

        # Update positions based on velocities
        for i in range(len(state['positions'])):
            predicted_state['positions'][i] += state['velocities'][i] * dt

        return predicted_state
```

### Sensor Fusion for Robust Perception

Combining multiple sensor modalities can mitigate individual sensor limitations:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class MultiSensorFusion:
    def __init__(self):
        self.camera_pose = None
        self.imu_data = None
        self.lidar_data = None
        self.fused_pose = None
        self.timestamp = 0

    def update_camera_pose(self, pose, timestamp):
        """
        Update pose estimate from camera (e.g., visual SLAM)
        """
        self.camera_pose = pose
        self.timestamp = max(self.timestamp, timestamp)

    def update_imu_data(self, linear_acc, angular_vel, timestamp):
        """
        Update IMU measurements
        """
        self.imu_data = {
            'linear_acc': linear_acc,
            'angular_vel': angular_vel
        }
        self.timestamp = max(self.timestamp, timestamp)

    def update_lidar_pose(self, pose, timestamp):
        """
        Update pose estimate from LiDAR (e.g., ICP)
        """
        self.lidar_data = pose
        self.timestamp = max(self.timestamp, timestamp)

    def fuse_poses(self, weights=None):
        """
        Fuse multiple pose estimates with weighted averaging
        """
        if weights is None:
            weights = {'camera': 0.4, 'lidar': 0.4, 'imu': 0.2}

        poses = []
        valid_weights = []

        if self.camera_pose is not None:
            poses.append(self.camera_pose)
            valid_weights.append(weights['camera'])

        if self.lidar_data is not None:
            poses.append(self.lidar_data)
            valid_weights.append(weights['lidar'])

        # Normalize weights
        total_weight = sum(valid_weights)
        if total_weight > 0:
            valid_weights = [w / total_weight for w in valid_weights]

        # Compute weighted average of poses
        if len(poses) > 0:
            fused_position = np.zeros(3)
            fused_rotation = np.eye(3)

            for i, pose in enumerate(poses):
                pos = pose[:3]
                rot_matrix = R.from_quat(pose[3:]).as_matrix()

                fused_position += valid_weights[i] * pos
                fused_rotation += valid_weights[i] * rot_matrix

            # Convert rotation back to quaternion
            fused_rotation = R.from_matrix(fused_rotation).as_quat()

            self.fused_pose = np.concatenate([fused_position, fused_rotation])

        return self.fused_pose
```

### Sensor Calibration and Alignment

Proper calibration is essential for accurate sensor data:

```python
class SensorCalibrator:
    def __init__(self):
        self.camera_intrinsics = None
        self.camera_distortion = None
        self.imu_bias = np.zeros(6)  # 3 for accel, 3 for gyro
        self.extrinsics = {}  # Transform between sensors

    def calibrate_camera(self, calibration_images, pattern_size=(9, 6)):
        """
        Calibrate camera intrinsics and distortion
        """
        import cv2
        import numpy as np

        # Prepare object points
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane

        for img in calibration_images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

            # If found, add object points, image points
            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)

        # Perform calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        self.camera_intrinsics = mtx
        self.camera_distortion = dist

        return ret, mtx, dist, rvecs, tvecs

    def calibrate_imu_bias(self, static_data_duration=30):
        """
        Calibrate IMU bias by averaging static measurements
        """
        import time

        print("Starting IMU bias calibration. Keep robot stationary...")
        start_time = time.time()

        accel_samples = []
        gyro_samples = []

        while time.time() - start_time < static_data_duration:
            # Get current IMU readings
            accel, gyro = self.get_current_imu_readings()
            accel_samples.append(accel)
            gyro_samples.append(gyro)
            time.sleep(0.01)  # 100Hz sampling

        # Calculate bias as mean of samples
        self.imu_bias[:3] = np.mean(accel_samples, axis=0)  # Accelerometer bias
        # Expected accelerometer reading is [0, 0, 9.81] in gravity direction
        self.imu_bias[:3] -= np.array([0, 0, 9.81])

        self.imu_bias[3:] = np.mean(gyro_samples, axis=0)  # Gyroscope bias

        print(f"IMU bias calibrated: {self.imu_bias}")
        return self.imu_bias

    def get_current_imu_readings(self):
        """
        Get current IMU readings (placeholder implementation)
        """
        # This would interface with actual IMU hardware
        return np.random.random(3), np.random.random(3)
```

## Deployment on Jetson Edge Devices

### Jetson Platform Considerations

NVIDIA Jetson platforms provide powerful edge computing for robotics, but require optimization for efficient deployment:

```python
import jetson.inference
import jetson.utils
import numpy as np
import torch

class JetsonDeployment:
    def __init__(self, model_path, input_shape=(224, 224)):
        self.model_path = model_path
        self.input_shape = input_shape
        self.model = None
        self.tensorrt_model = None

    def optimize_for_jetson(self):
        """
        Optimize model for Jetson deployment using TensorRT
        """
        import torch
        import torch_tensorrt

        # Load PyTorch model
        self.model = torch.jit.load(self.model_path)
        self.model.eval()

        # Convert to TensorRT for acceleration
        example_input = torch.randn(1, 3, *self.input_shape).cuda()

        self.tensorrt_model = torch_tensorrt.compile(
            self.model,
            inputs=[example_input],
            enabled_precisions={torch.float, torch.half},  # Use FP16 for speed
            workspace_size=1 << 25  # 32MB workspace
        )

        return self.tensorrt_model

    def deploy_policy(self, policy_network):
        """
        Deploy policy network to Jetson with optimizations
        """
        # Convert to TensorRT
        optimized_policy = self.optimize_for_jetson()

        # Set up input/output buffers
        self.input_buffer = torch.zeros(1, *self.input_shape).cuda()
        self.output_buffer = torch.zeros(policy_network.output_size).cuda()

        return optimized_policy

    def run_inference(self, input_data):
        """
        Run optimized inference on Jetson
        """
        with torch.no_grad():
            # Preprocess input
            input_tensor = self.preprocess_input(input_data)

            # Run inference
            output = self.tensorrt_model(input_tensor)

            return output.cpu().numpy()

    def preprocess_input(self, input_data):
        """
        Preprocess input data for Jetson inference
        """
        # Convert to tensor and move to GPU
        if not isinstance(input_data, torch.Tensor):
            input_tensor = torch.from_numpy(input_data).float().cuda()
        else:
            input_tensor = input_data.float().cuda()

        # Ensure correct shape and normalize if needed
        if input_tensor.dim() == 3:
            input_tensor = input_tensor.unsqueeze(0)  # Add batch dimension

        return input_tensor
```

### Real-Time Performance Optimization

```python
class RealTimeOptimizer:
    def __init__(self, target_frequency=50):  # 50Hz target
        self.target_frequency = target_frequency
        self.target_period = 1.0 / target_frequency
        self.last_execution_time = 0
        self.control_thread = None

    def optimize_policy_execution(self, policy_func):
        """
        Optimize policy execution for real-time performance
        """
        import threading
        import time

        def optimized_execution():
            while True:
                start_time = time.time()

                # Execute policy
                action = policy_func()

                # Calculate execution time
                execution_time = time.time() - start_time
                self.last_execution_time = execution_time

                # Calculate sleep time to maintain target frequency
                sleep_time = self.target_period - execution_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"Warning: Execution time ({execution_time:.3f}s) exceeds target period ({self.target_period:.3f}s)")

        # Start control thread
        self.control_thread = threading.Thread(target=optimized_execution)
        self.control_thread.daemon = True
        self.control_thread.start()

        return optimized_execution

    def set_cpu_affinity(self, cpu_cores=[0, 1]):
        """
        Set CPU affinity to dedicated cores for deterministic performance
        """
        import os
        import psutil

        current_process = psutil.Process(os.getpid())
        current_process.cpu_affinity(cpu_cores)
        print(f"Set CPU affinity to cores: {cpu_cores}")

    def enable_jetson_performance_mode(self):
        """
        Enable maximum performance mode on Jetson
        """
        import subprocess

        try:
            # Set Jetson to maximum performance mode
            subprocess.run(['sudo', 'nvpmodel', '-m', '0'], check=True)
            subprocess.run(['sudo', 'jetson_clocks'], check=True)
            print("Jetson performance mode enabled")
        except subprocess.CalledProcessError:
            print("Failed to set Jetson performance mode")
```

## Failure Modes and Mitigation Strategies

### Common Sim-to-Real Failure Modes

#### Dynamics Mismatch Failures
- **Underactuated Systems**: Simulated robots may appear more capable than real ones
- **Unmodeled Flexibility**: Structural compliance causing unexpected behavior
- **Actuator Limitations**: Torque/speed limits not captured in simulation

#### Perception Failures
- **Lighting Sensitivity**: Policies trained under specific lighting failing under different conditions
- **Texture Dependency**: Policies that rely on specific visual textures
- **Sensor Noise**: Real sensor noise causing policy confusion

#### Environmental Failures
- **Surface Properties**: Different friction coefficients causing locomotion failures
- **Obstacle Variations**: Simulated obstacles not representing real complexity
- **External Disturbances**: Unmodeled forces affecting robot behavior

### Failure Detection and Recovery

```python
class FailureDetector:
    def __init__(self, safety_thresholds):
        self.safety_thresholds = safety_thresholds
        self.anomaly_detector = None
        self.recovery_strategies = []

    def detect_failure(self, robot_state, action, reward):
        """
        Detect potential failures based on state, action, and reward patterns
        """
        failures = []

        # Check joint limits
        if self._check_joint_limits(robot_state):
            failures.append('joint_limit_violation')

        # Check for abnormal energy consumption
        if self._check_energy_anomaly(robot_state):
            failures.append('energy_anomaly')

        # Check for unexpected state changes
        if self._check_state_anomaly(robot_state):
            failures.append('state_anomaly')

        # Check reward anomalies (sudden drops)
        if self._check_reward_anomaly(reward):
            failures.append('reward_anomaly')

        return failures

    def _check_joint_limits(self, state):
        """
        Check if joints are approaching or exceeding limits
        """
        joint_positions = state['joint_positions']
        joint_limits = state['joint_limits']

        for i, (pos, limits) in enumerate(zip(joint_positions, joint_limits)):
            if pos < limits[0] or pos > limits[1]:
                return True
            # Check if approaching limits (within 10% of range)
            range_size = limits[1] - limits[0]
            if abs(pos - limits[0]) < 0.1 * range_size or abs(pos - limits[1]) < 0.1 * range_size:
                return True

        return False

    def _check_energy_anomaly(self, state):
        """
        Check for abnormal energy consumption patterns
        """
        joint_velocities = state['joint_velocities']
        joint_torques = state['joint_torques']

        # Calculate power consumption
        power = np.sum(np.abs(joint_torques * joint_velocities))

        # Compare to expected range
        if power > self.safety_thresholds.get('max_power', 100):
            return True

        return False

    def _check_state_anomaly(self, state):
        """
        Check for unexpected state patterns using statistical methods
        """
        # This could use machine learning models trained on normal behavior
        current_features = self._extract_state_features(state)

        if self.anomaly_detector:
            anomaly_score = self.anomaly_detector.score_samples([current_features])
            if anomaly_score[0] < self.safety_thresholds.get('anomaly_threshold', -1.0):
                return True

        return False

    def _check_reward_anomaly(self, reward):
        """
        Check for sudden drops in reward that might indicate failure
        """
        # Compare to recent reward history
        if hasattr(self, 'recent_rewards'):
            if len(self.recent_rewards) > 10:
                avg_recent = np.mean(self.recent_rewards[-10:])
                if reward < avg_recent * 0.5:  # Reward dropped by 50%
                    return True
        else:
            self.recent_rewards = []

        self.recent_rewards.append(reward)
        # Keep only recent history
        if len(self.recent_rewards) > 100:
            self.recent_rewards = self.recent_rewards[-50:]

        return False
```

### Safe Recovery Strategies

```python
class SafeRecovery:
    def __init__(self, robot_controller):
        self.controller = robot_controller
        self.recovery_strategies = {
            'emergency_stop': self.emergency_stop,
            'safe_pose': self.move_to_safe_pose,
            'return_to_home': self.return_to_home_position,
            'contact_recovery': self.contact_recovery_manipulation
        }

    def execute_recovery(self, failure_type, current_state):
        """
        Execute appropriate recovery strategy based on failure type
        """
        if failure_type in self.recovery_strategies:
            return self.recovery_strategies[failure_type](current_state)
        else:
            # Default to emergency stop
            return self.emergency_stop(current_state)

    def emergency_stop(self, state):
        """
        Immediately stop all robot motion
        """
        print("Executing emergency stop...")
        self.controller.set_joint_velocities(np.zeros_like(state['joint_positions']))
        return True

    def move_to_safe_pose(self, state):
        """
        Move robot to a predefined safe pose
        """
        print("Moving to safe pose...")
        safe_joint_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Example safe pose

        # Use trajectory planning to move safely
        trajectory = self._plan_trajectory_to_pose(
            current_state['joint_positions'],
            safe_joint_positions
        )

        for waypoint in trajectory:
            self.controller.set_joint_positions(waypoint)
            time.sleep(0.01)  # 100Hz control

        return True

    def return_to_home_position(self, state):
        """
        Return robot to home/base position
        """
        print("Returning to home position...")
        home_position = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0])  # Example home pose

        trajectory = self._plan_trajectory_to_pose(
            state['joint_positions'],
            home_position
        )

        for waypoint in trajectory:
            self.controller.set_joint_positions(waypoint)
            time.sleep(0.01)

        return True

    def contact_recovery_manipulation(self, state):
        """
        Recovery strategy for manipulation tasks with unexpected contact
        """
        print("Executing contact recovery...")
        # Release grasp if holding something
        if state.get('gripper_closed', False):
            self.controller.open_gripper()

        # Move away from contact point slowly
        current_pos = state['end_effector_position']
        contact_normal = state.get('contact_normal', np.array([0, 0, 1]))

        # Move opposite to contact normal
        new_pos = current_pos - 0.05 * contact_normal  # 5cm away
        self.controller.move_to_position(new_pos)

        return True

    def _plan_trajectory_to_pose(self, start_pos, end_pos, steps=50):
        """
        Plan simple linear trajectory between two joint configurations
        """
        trajectory = []
        for i in range(steps + 1):
            t = i / steps
            waypoint = start_pos + t * (end_pos - start_pos)
            trajectory.append(waypoint)
        return trajectory
```

## Best Practices for Successful Transfer

### Gradual Domain Randomization

Start with minimal randomization and gradually increase as the policy improves:

```python
class GradualDomainRandomization:
    def __init__(self, base_env, max_randomization_steps=1000000):
        self.base_env = base_env
        self.max_steps = max_randomization_steps
        self.current_step = 0
        self.randomization_schedule = {
            'mass_multiplier': {'start': 0.9, 'end': 1.1, 'type': 'linear'},
            'friction': {'start': 0.8, 'end': 1.2, 'type': 'linear'},
            'lighting': {'start': 0.9, 'end': 1.1, 'type': 'linear'}
        }

    def get_current_randomization_params(self):
        """
        Get current randomization parameters based on training progress
        """
        progress = min(1.0, self.current_step / self.max_steps)

        params = {}
        for param_name, schedule in self.randomization_schedule.items():
            if schedule['type'] == 'linear':
                start_val = schedule['start']
                end_val = schedule['end']
                current_range = start_val + progress * (end_val - start_val)
                # Calculate symmetric range around 1.0
                params[param_name] = (2.0 - current_range, current_range)
            elif schedule['type'] == 'exponential':
                # Exponential increase in randomization
                factor = schedule['start'] + (schedule['end'] - schedule['start']) * (progress ** 2)
                params[param_name] = (2.0 - factor, factor)

        return params

    def step(self):
        """
        Increment step counter and update randomization
        """
        self.current_step += 1
        return self.get_current_randomization_params()
```

### System Identification Integration

Combine system identification with RL for better sim-to-real transfer:

```python
class SystemIDIntegratedRL:
    def __init__(self, env, system_id_frequency=1000):
        self.env = env
        self.system_id_frequency = system_id_frequency
        self.system_id_model = None
        self.dynamics_params = {}
        self.update_counter = 0

    def update_dynamics_model(self, experience_buffer):
        """
        Update dynamics model based on collected experience
        """
        # Extract state-action-reward transitions
        states = [exp.state for exp in experience_buffer]
        actions = [exp.action for exp in experience_buffer]
        next_states = [exp.next_state for exp in experience_buffer]

        # Identify system dynamics parameters
        self.dynamics_params = self._identify_dynamics(states, actions, next_states)

        # Update environment with new parameters
        self._update_env_dynamics(self.dynamics_params)

    def _identify_dynamics(self, states, actions, next_states):
        """
        Identify system dynamics from experience data
        """
        # Use system identification techniques
        # This is a simplified example - real implementation would be more complex
        import scipy.optimize

        def dynamics_error(params):
            total_error = 0
            for s, a, ns in zip(states, actions, next_states):
                predicted_ns = self._simulate_dynamics(s, a, params)
                error = np.sum((predicted_ns - ns) ** 2)
                total_error += error
            return total_error

        # Optimize dynamics parameters
        initial_params = self._get_initial_dynamics_params()
        result = scipy.optimize.minimize(dynamics_error, initial_params)

        return result.x

    def _simulate_dynamics(self, state, action, params):
        """
        Simulate system dynamics with given parameters
        """
        # This would implement the actual dynamics model
        # For example: q_dot = f(q, q_dot, tau, params)
        pass

    def _get_initial_dynamics_params(self):
        """
        Get initial guess for dynamics parameters
        """
        # Return initial parameter values
        return np.array([1.0, 1.0, 0.1])  # Example: mass, damping, etc.

    def _update_env_dynamics(self, params):
        """
        Update environment dynamics with identified parameters
        """
        for i, param in enumerate(params):
            self.env.update_dynamics_parameter(i, param)
```

## Key Claims Requiring Citations

1. Domain randomization significantly reduces the sim-to-real transfer gap by training policies across varied environmental conditions (Citation needed - see references.md)

2. Latency compensation techniques are essential for maintaining control performance in real robotic systems (Citation needed - see references.md)

3. Sensor fusion approaches improve robustness by combining complementary sensing modalities (Citation needed - see references.md)

4. Jetson edge computing platforms enable deployment of complex robotic policies with real-time performance (Citation needed - see references.md)

5. Failure detection and recovery mechanisms are critical for safe real-world robotic deployment (Citation needed - see references.md)

6. Gradual domain randomization improves training stability compared to immediate full randomization (Citation needed - see references.md)

7. System identification integration helps align simulation and reality by learning real system parameters (Citation needed - see references.md)

## Reproducibility Notes

- All examples assume ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS
- Required packages: NumPy, SciPy, PyTorch, OpenCV, Jetson Inference
- Hardware requirements: NVIDIA Jetson platform (AGX Orin, Xavier NX, or Nano)
- Training times vary significantly based on domain randomization settings
- Proper sensor calibration is essential for successful transfer

### Diagrams

![Reality Gap Visualization](/diagrams/reality-gap-visualization.svg)

*Figure 9.1: Visualization of the reality gap showing differences between simulation and real-world conditions.*

![Domain Randomization Process](/diagrams/domain-randomization-process.svg)

*Figure 9.2: Domain randomization process for improving sim-to-real transfer in robotics.*

## Summary

This chapter covered the critical challenges and techniques for sim-to-real transfer in robotics. We explored domain randomization methods, latency and sensor mismatch mitigation, Jetson deployment strategies, and failure detection and recovery approaches. The next chapter will address the future of Physical AI, examining scaling challenges, ethical considerations, and research frontiers in humanoid robotics.

---

## References

For full citations, see [References](/docs/references.md).