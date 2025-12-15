# ROS 2 Architecture Diagram

## Description
This diagram illustrates the ROS 2 system architecture showing nodes, topics, services, and actions.

## Components to Include
- **Nodes**: Multiple rectangular boxes representing different ROS 2 nodes
- **Topics**: Arrowed lines showing publish-subscribe communication
- **Services**: Bidirectional arrows showing request-response communication
- **Actions**: Three-line communication pattern showing goal-feedback-result
- **DDS Middleware**: Background layer showing the Data Distribution Service
- **Executors**: Components managing node execution
- **Parameter Server**: Centralized parameter management

## Visual Elements
- Different colors for different types of communication
- Clear labels for each component
- Example node names: sensor_node, controller_node, planner_node
- QoS policy indicators where appropriate
- Intra-process vs inter-process communication distinction

## Use Case
This diagram should help readers understand how ROS 2 components interact and communicate with each other in a distributed system.