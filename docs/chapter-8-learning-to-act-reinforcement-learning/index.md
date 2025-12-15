title: "Chapter 8 - Learning to Act: Reinforcement Learning for Robots"
sidebar_position: 8

# Learning to Act: Reinforcement Learning for Robots

## Learning Objectives

After completing this chapter, readers will be able to:
1. Understand the fundamentals of reinforcement learning in continuous control environments
2. Apply RL algorithms to robotic manipulation and locomotion tasks
3. Design simulation-based training pipelines for robotic policies
4. Analyze and mitigate challenges in sim-to-real policy transfer
5. Implement safety constraints in robotic RL systems

## Introduction to Continuous Control RL

Reinforcement Learning (RL) represents a paradigm shift from supervised learning, where agents learn to interact with environments through trial and error to maximize cumulative rewards. In robotics, this approach is particularly powerful for learning complex behaviors that are difficult to program explicitly.

### Continuous Action Spaces

Unlike discrete control problems with finite action sets, robotic systems operate in continuous action spaces where the agent must output precise motor commands. This presents unique challenges:

- **Infinite Action Space**: Instead of selecting from a finite set of actions, the agent outputs continuous values for joint velocities, torques, or positions
- **Stochastic Policies**: Continuous control often requires stochastic policies that output probability distributions over actions rather than deterministic decisions
- **Sample Efficiency**: Continuous control problems typically require more samples to learn effective policies

### Key RL Algorithms for Robotics

#### Deep Deterministic Policy Gradient (DDPG)

DDPG is an actor-critic algorithm designed for continuous control that learns deterministic policies:

```python
import torch
import torch.nn as nn
import numpy as np

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()

        self.l1 = nn.Linear(state_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, action_dim)

        self.max_action = max_action

    def forward(self, state):
        a = torch.relu(self.l1(state))
        a = torch.relu(self.l2(a))
        return self.max_action * torch.tanh(self.l3(a))

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()

        self.l1 = nn.Linear(state_dim + action_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], 1)
        q = torch.relu(self.l1(sa))
        q = torch.relu(self.l2(q))
        q = self.l3(q)
        return q
```

#### Twin Delayed DDPG (TD3)

TD3 addresses overestimation bias in DDPG through three key improvements:
1. Clipped Double-Q Learning: Uses two critics and takes the minimum
2. Target Policy Smoothing: Adds noise to target actions
3. Delayed Policy Updates: Updates actor less frequently

#### Soft Actor-Critic (SAC)

SAC is an off-policy actor-critic algorithm that maximizes both expected return and entropy:

```python
import torch
import torch.nn as nn
import torch.nn.functional as F

class SACActor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(SACActor, self).__init__()

        self.l1 = nn.Linear(state_dim, 256)
        self.l2 = nn.Linear(256, 256)

        self.mean = nn.Linear(256, action_dim)
        self.log_std = nn.Linear(256, action_dim)

        self.max_action = max_action

    def forward(self, state):
        a = F.relu(self.l1(state))
        a = F.relu(self.l2(a))

        mean = self.mean(a)
        log_std = self.log_std(a)
        log_std = torch.clamp(log_std, min=-20, max=2)

        return mean, log_std

    def sample(self, state):
        mean, log_std = self.forward(state)
        std = log_std.exp()
        normal = torch.distributions.Normal(mean, std)

        x_t = normal.rsample()
        y_t = torch.tanh(x_t)
        action = y_t * self.max_action

        # Compute log probability
        log_prob = normal.log_prob(x_t)
        log_prob -= torch.log(self.max_action * (1 - y_t.pow(2)) + 1e-6)
        log_prob = log_prob.sum(1, keepdim=True)

        return action, log_prob
```

## Simulation-Based Training Pipelines

### Gym and Custom Environments

Robotic RL typically uses environments based on the OpenAI Gym interface:

```python
import gym
from gym import spaces
import numpy as np

class RoboticManipulationEnv(gym.Env):
    def __init__(self):
        super(RoboticManipulationEnv, self).__init__()

        # Define action and observation spaces
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(7,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(30,), dtype=np.float32
        )

        # Robot state: joint positions, velocities, end-effector pose
        # Object state: position, orientation, goal position

    def reset(self):
        # Reset robot to initial configuration
        # Place object at random position
        # Return initial observation
        observation = self._get_observation()
        return observation

    def step(self, action):
        # Execute action in simulation
        self._apply_action(action)

        # Update simulation
        self._update_simulation()

        # Compute reward and check termination
        reward = self._compute_reward()
        done = self._is_done()
        info = {}

        observation = self._get_observation()
        return observation, reward, done, info

    def _get_observation(self):
        # Concatenate joint states, object states, goal states
        obs = np.concatenate([
            self.joint_positions,
            self.joint_velocities,
            self.end_effector_pose,
            self.object_pose,
            self.goal_pose
        ])
        return obs

    def _compute_reward(self):
        # Compute dense reward based on task progress
        # e.g., negative distance to goal, success bonus
        distance_to_goal = np.linalg.norm(
            self.end_effector_pos - self.goal_pos
        )
        reward = -distance_to_goal  # Dense reward

        if self._is_success():
            reward += 100  # Success bonus

        return reward
```

### Isaac Gym Integration

NVIDIA Isaac Gym provides GPU-accelerated physics simulation:

```python
import isaacgym
from isaacgym import gymapi, gymtorch
import torch

class IsaacGymRobotEnv:
    def __init__(self, cfg):
        # Initialize physics engine
        self.gym = gymapi.acquire_gym()
        self.sim = self.gym.create_sim(
            device_id=0,
            gpu_id=0,
            type=gymapi.SIM_PHYSX,
            params=cfg['sim_params']
        )

        # Create ground plane
        plane_params = gymapi.PlaneParams()
        self.gym.add_ground(self.sim, plane_params)

        # Create environment instances
        self._create_envs()

    def _create_envs(self):
        # Load robot asset
        asset_root = cfg['asset']['root']
        asset_file = cfg['asset']['file']

        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        asset_options.flip_visual_attachments = False
        asset_options.collapse_fixed_joints = True
        asset_options.disable_gravity = False
        asset_options.thickness = 0.001
        asset_options.angular_damping = 0.01
        asset_options.linear_damping = 0.01

        robot_asset = self.gym.load_asset(
            self.sim, asset_root, asset_file, asset_options
        )

        # Create multiple parallel environments
        num_envs = cfg['num_envs']
        spacing = cfg['env_spacing']

        for i in range(num_envs):
            env = self.gym.create_env(
                self.sim,
                gymapi.Vec3(-spacing, 0.0, -spacing),
                gymapi.Vec3(spacing, spacing, spacing),
                1
            )

            # Add robot to environment
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0.0, 0.0, 0.0)

            self.gym.create_actor(
                env, robot_asset, pose, "robot", i, 1
            )

    def step(self, actions):
        # Apply actions to all environments
        self.gym.set_dof_position_targets(self.envs_ptr, actions)

        # Step simulation
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Get observations and rewards
        obs = self._get_observations()
        rew = self._get_rewards()
        dones = self._get_dones()

        return obs, rew, dones, {}
```

### Training Loops

A typical training loop for robotic RL:

```python
def train_robotic_rl_agent(env, agent, total_timesteps):
    obs = env.reset()
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    for t in range(total_timesteps):
        # Select action randomly or according to policy
        if t < cfg['start_timesteps']:
            action = env.action_space.sample()
        else:
            action = agent.select_action(obs)

            # Add noise to action for exploration
            noise = np.random.normal(0, cfg['exploration_noise'], size=action.shape)
            action = (action + noise).clip(-1, 1)

        # Perform action
        new_obs, reward, done, info = env.step(action)
        episode_reward += reward
        episode_timesteps += 1

        # Store data in replay buffer
        agent.replay_buffer.add(obs, action, new_obs, reward, done)

        # Train agent
        if t >= cfg['start_timesteps']:
            agent.train(batch_size=cfg['batch_size'])

        # Reset environment if episode is done
        if done:
            print(f"Total T: {t+1} Episode Num: {episode_num+1} Episode T: {episode_timesteps} Reward: {episode_reward:.3f}")
            obs = env.reset()
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1
        else:
            obs = new_obs

    return agent
```

## Policy Transfer Challenges

### The Reality Gap

The reality gap represents the fundamental challenge of transferring policies trained in simulation to the real world. Key factors include:

#### Dynamics Mismatch
- **Inertia**: Real robot dynamics may differ from simulation
- **Friction**: Joint friction and external forces not modeled in simulation
- **Actuator Response**: Delayed or non-linear actuator responses
- **Sensor Noise**: Real sensors have noise patterns not present in simulation

#### Environmental Factors
- **Lighting**: Visual perception can be affected by lighting changes
- **Surface Properties**: Floor friction, object textures differ from simulation
- **External Disturbances**: Unmodeled forces from environment

### Domain Randomization

Domain randomization is a key technique to bridge the sim-to-real gap:

```python
class DomainRandomizedEnv:
    def __init__(self, base_env):
        self.base_env = base_env
        self.param_ranges = {
            'mass_multiplier': (0.8, 1.2),
            'friction': (0.1, 0.9),
            'restitution': (0.0, 0.5),
            'gravity': (-11.0, -9.0),
            'lighting': (0.5, 2.0)  # Brightness multiplier
        }

    def reset(self):
        # Randomize physical parameters
        self._randomize_dynamics()
        self._randomize_visuals()
        return self.base_env.reset()

    def _randomize_dynamics(self):
        # Randomize robot and object properties
        for param, (min_val, max_val) in self.param_ranges.items():
            if param in ['mass_multiplier', 'friction', 'restitution', 'gravity']:
                random_val = np.random.uniform(min_val, max_val)
                self._apply_dynamics_parameter(param, random_val)

    def _randomize_visuals(self):
        # Randomize lighting and textures
        brightness = np.random.uniform(0.5, 2.0)
        self._set_lighting_brightness(brightness)

        # Randomize object textures
        texture_id = np.random.randint(0, len(self.texture_pool))
        self._apply_texture(texture_id)
```

### System Identification and System Dynamics

Accurate modeling of system dynamics is crucial:

```python
def identify_robot_dynamics(robot, trajectories):
    """
    Identify robot dynamics parameters from collected data
    """
    # Collect data: (q, q_dot, q_ddot, tau) tuples
    # where q = joint positions, q_dot = velocities, q_ddot = accelerations, tau = torques

    # Lagrangian dynamics: M(q)q_ddot + C(q, q_dot)q_dot + g(q) = tau
    # where M = inertia matrix, C = Coriolis matrix, g = gravity vector

    # Use least squares to identify parameters
    # tau = Y(theta) * theta_hat (regression form)

    # For rigid body dynamics:
    # theta = [m1, m2, ..., I1, I2, ..., com1, com2, ...]

    Y_matrix = construct_regression_matrix(trajectories)
    tau_vector = trajectories['torques']

    # Solve: theta_hat = (Y^T * Y)^(-1) * Y^T * tau
    estimated_params = np.linalg.lstsq(Y_matrix, tau_vector, rcond=None)[0]

    return estimated_params
```

## Safety Constraints in RL

### Safe RL Framework

Safe RL incorporates constraints to prevent dangerous behavior:

```python
def safe_rl_objective(reward, constraint_violation_cost, safety_penalty):
    """
    Modified objective that penalizes constraint violations
    """
    # Original reward minus safety penalties
    return reward - constraint_violation_cost - safety_penalty

class ConstrainedRLAgent:
    def __init__(self, base_agent, constraint_functions):
        self.base_agent = base_agent
        self.constraint_functions = constraint_functions
        self.safety_violations = 0

    def compute_safe_action(self, state):
        # Get action from base policy
        action = self.base_agent.select_action(state)

        # Check constraints
        for constraint_func in self.constraint_functions:
            if not constraint_func(state, action):
                # Apply safety intervention
                action = self._safe_intervention(state, action)
                self.safety_violations += 1
                break

        return action

    def _safe_intervention(self, state, action):
        # Implement safety fallback strategy
        # e.g., emergency stop, safe pose, minimal action
        return np.zeros_like(action)
```

### Control Barrier Functions (CBFs)

CBFs provide formal safety guarantees:

```python
class ControlBarrierFunction:
    def __init__(self, safe_set_function, alpha_function):
        self.h = safe_set_function  # h(x) >= 0 defines safe set
        self.alpha = alpha_function  # class-K function

    def is_safe(self, state):
        return self.h(state) >= 0

    def safe_control_filter(self, state, nominal_control):
        """
        Filter control to ensure safety constraints
        """
        # Constraint: Lf_h + Lg_h * u + alpha(h) >= 0
        # where Lf_h and Lg_h are Lie derivatives
        Lf_h = self._lie_derivative_f(state)
        Lg_h = self._lie_derivative_g(state)

        # Solve: Lg_h * u >= -(Lf_h + alpha(h(state)))
        safety_constraint = -(Lf_h + self.alpha(self.h(state)))

        if Lg_h != 0:
            min_control = safety_constraint / Lg_h
            if Lg_h > 0:
                return max(nominal_control, min_control)
            else:
                return min(nominal_control, min_control)
        else:
            return nominal_control
```

### Human Safety Considerations

For humanoid robots operating near humans:

```python
def human_aware_safety_constraint(robot_state, human_state, action):
    """
    Ensure robot actions maintain safe distance from humans
    """
    robot_pos = robot_state['position']
    human_pos = human_state['position']

    min_safe_distance = 0.5  # meters

    distance = np.linalg.norm(robot_pos - human_pos)

    # If currently safe and action maintains safety
    if distance > min_safe_distance:
        # Predict next position with action
        predicted_robot_pos = predict_robot_motion(robot_state, action)
        predicted_distance = np.linalg.norm(predicted_robot_pos - human_pos)
        return predicted_distance > min_safe_distance

    return False  # Unsafe if currently too close
```

## Practical Implementation Examples

### Quadruped Locomotion with RL

```python
class QuadrupedLocomotionRL:
    def __init__(self, robot_env):
        self.env = robot_env
        self.terrain_generator = TerrainGenerator()
        self.gait_pattern = GaitPattern()

    def compute_locomotion_reward(self, robot_state, prev_robot_state):
        """
        Compute reward for forward locomotion
        """
        # Forward velocity reward
        forward_vel = robot_state['base_velocity'][0]
        forward_reward = max(0, forward_vel)  # Only reward forward motion

        # Smoothness reward
        joint_velocities = robot_state['joint_velocities']
        smoothness_penalty = np.sum(joint_velocities ** 2) * 0.001

        # Energy efficiency
        joint_torques = robot_state['joint_torques']
        energy_penalty = np.sum(np.abs(joint_torques * joint_velocities)) * 0.0001

        # Stability (keep body level)
        body_orientation = robot_state['base_orientation']
        roll, pitch = body_orientation[0], body_orientation[1]
        stability_reward = np.exp(-5 * (roll**2 + pitch**2))

        # Stay upright
        body_height = robot_state['base_position'][2]
        target_height = 0.5
        height_penalty = abs(body_height - target_height) * 0.1

        total_reward = (forward_reward - smoothness_penalty -
                       energy_penalty + stability_reward - height_penalty)

        return total_reward

    def train_locomotion_policy(self):
        """
        Train locomotion policy with curriculum learning
        """
        curricula = [
            'flat_terrain', 'rough_terrain',
            'sloped_terrain', 'obstacles'
        ]

        for curriculum_phase in curricula:
            self.terrain_generator.set_terrain_type(curriculum_phase)
            # Train for this phase
            self._train_phase(epochs=1000)
```

### Robotic Manipulation Learning

```python
class ManipulationRLAgent:
    def __init__(self, env):
        self.env = env
        self.skill_library = {}
        self.curriculum = [
            'reach_to_point',
            'grasp_object',
            'lift_object',
            'move_object',
            'place_object'
        ]

    def hierarchical_policy(self, task_goal):
        """
        Use hierarchical policy to break down complex tasks
        """
        # Identify subtasks needed to achieve goal
        subtasks = self._plan_subtasks(task_goal)

        # Execute each subtask with appropriate skill
        for subtask in subtasks:
            skill = self.skill_library.get(subtask, self.default_skill)
            success = skill.execute(self.env)
            if not success:
                return False

        return True

    def _plan_subtasks(self, goal):
        """
        Plan sequence of subtasks to achieve goal
        """
        # Example: Move object A to location B
        # Subtasks: Reach A, Grasp A, Lift A, Move to B, Place A
        if 'move_object' in goal:
            return ['reach_object', 'grasp_object', 'lift_object',
                   'navigate_to_location', 'place_object']
        return [goal]
```

## Key Claims Requiring Citations

1. Reinforcement learning enables robots to learn complex behaviors that are difficult to program explicitly through trial and error (Citation needed - see references.md)

2. Continuous control RL algorithms like DDPG and SAC are essential for robotic systems operating in continuous action spaces (Citation needed - see references.md)

3. Domain randomization techniques significantly improve sim-to-real transfer by training policies across varied environmental conditions (Citation needed - see references.md)

4. Safety constraints and control barrier functions are critical for preventing dangerous behavior in robotic RL systems (Citation needed - see references.md)

5. GPU-accelerated simulation enables efficient training of complex robotic policies through parallel environment execution (Citation needed - see references.md)

6. Hierarchical RL approaches improve sample efficiency by decomposing complex tasks into manageable subtasks (Citation needed - see references.md)

7. Curriculum learning strategies accelerate training by starting with simpler tasks and gradually increasing complexity (Citation needed - see references.md)

## Reproducibility Notes

- All examples assume ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS
- Required packages: PyTorch, Isaac Gym, OpenAI Gym, NumPy
- Hardware requirements: NVIDIA GPU with CUDA support for Isaac Gym
- Training times vary significantly based on environment complexity (hours to days)
- Simulation environments require accurate robot dynamics models

### Diagrams

![RL Algorithms Comparison](/diagrams/rl-algorithms-comparison.svg)

*Figure 8.1: Comparison of different reinforcement learning algorithms for robotics including DDPG, TD3, and SAC.*

![Sim-to-Real Transfer](/diagrams/sim-to-real-transfer.svg)

*Figure 8.2: Framework for sim-to-real transfer in reinforcement learning showing domain randomization and system identification techniques.*

## Summary

This chapter explored reinforcement learning for robotics, covering continuous control algorithms, simulation-based training, policy transfer challenges, and safety considerations. We examined key algorithms like DDPG, TD3, and SAC for continuous action spaces, and discussed practical implementation challenges including the reality gap, domain randomization, and safe RL. The next chapter will address sim-to-real transfer challenges and deployment strategies for robotic systems trained in simulation.

---

## References

For full citations, see [References](/docs/references.md).