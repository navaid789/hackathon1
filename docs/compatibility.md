# Compatibility Requirements

This document outlines the system compatibility requirements for the Physical AI & Humanoid Robotics textbook content and examples.

## Operating System Compatibility

### Primary Target: Ubuntu 22.04 LTS
- **Minimum**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Recommended**: Ubuntu 22.04 LTS with latest updates
- **Kernel Version**: 5.15.x or higher recommended
- **Architecture**: x86_64 (AMD64)

### Secondary Targets (Compatible)
- Ubuntu 20.04 LTS (with caveats - see notes)
- Debian 11 (bullseye) - ROS 2 compatibility may vary

## ROS 2 Compatibility

### Primary Target: ROS 2 Humble Hawksbill
- **Distribution**: ROS 2 Humble Hawksbill (LTS)
- **Release**: ROS 2 Rolling Ridley → Humble Hawksbill migration completed 2022
- **Support Window**: Until May 2027 (LTS release)
- **Package Manager**: apt packages from packages.ros.org

### Compatible ROS 2 Versions
- ROS 2 Iron Irwini (for newer features)
- ROS 2 Jazzy Jalisco (upcoming LTS)

### Incompatible Versions
- ROS 1 (no bridge planned for textbook examples)
- ROS 2 Foxy Fitzroy (EOL May 2023)

## Hardware Requirements

### Minimum Specifications
- **CPU**: Intel i7-10700K or AMD Ryzen 7 3700X (8 cores, 16 threads)
- **RAM**: 32GB DDR4-3200 (minimum), 64GB recommended
- **GPU**: NVIDIA RTX 2080 (8GB VRAM minimum)
- **Storage**: 500GB NVMe SSD (1TB recommended for simulation)
- **Network**: Gigabit Ethernet, WiFi 6 (802.11ax) recommended

### Recommended Specifications
- **CPU**: Intel i9-12900K or AMD Ryzen 9 5900X
- **RAM**: 64GB DDR4-3600 or DDR5-4800
- **GPU**: NVIDIA RTX 3080/3090 or RTX 4080/4090 (24GB VRAM)
- **Storage**: 1TB+ NVMe SSD, additional 2TB for datasets
- **Network**: 2.5Gbps Ethernet, WiFi 6E

## Simulation Environment Compatibility

### NVIDIA Isaac Sim
- **Minimum**: Isaac Sim 2022.2.1
- **Recommended**: Isaac Sim 2023.1+
- **GPU Driver**: NVIDIA Driver 520+ (for RTX 40-series)
- **CUDA**: CUDA 11.8 or 12.0
- **Container**: Docker support for Isaac Sim 2023.1+

### Gazebo Compatibility
- **Primary**: Gazebo Garden (Fortress compatibility maintained)
- **Alternative**: Ignition Gazebo Harmonic (if available)
- **ROS Integration**: ros-gazebo-packages for Humble

### Unity Compatibility (for comparison)
- **Version**: Unity 2022.3 LTS (for long-term support)
- **System Requirements**: Compatible with Ubuntu 22.04 via Unity Hub
- **Limitations**: Not primary simulation environment for textbook

## Python and Development Environment

### Python Compatibility
- **Primary**: Python 3.10 (Ubuntu 22.04 default)
- **Supported**: Python 3.11 (with ROS 2 Humble patches)
- **Not Supported**: Python 3.12 (packages not ready)

### Development Tools
- **IDE**: VS Code with ROS extension, PyCharm Professional
- **Build Tools**: colcon, CMake 3.22+, Python setuptools
- **Virtual Environments**: venv, conda (Anaconda/Miniconda 23.x)

## Network and Deployment

### Local Development
- **Docker**: Docker Engine 20.10+ with NVIDIA Container Toolkit
- **Port Requirements**: 8080 (Docusaurus), 11311 (ROS master), various for Isaac Sim
- **Firewall**: Outbound HTTPS (443) for package managers and updates

### Cloud Deployment Considerations
- **GitHub Pages**: Static site hosting (textbook content)
- **ROS Bridge**: Not applicable (textbook is documentation-only)
- **Simulation**: Requires local execution due to real-time requirements

## Validation Commands

To verify system compatibility, run:

```bash
# Check Ubuntu version
lsb_release -a

# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version

# Check GPU and CUDA (if NVIDIA)
nvidia-smi
nvcc --version

# Check Python version
python3 --version

# Check available RAM
free -h
```

## Known Issues and Workarounds

### Ubuntu 20.04 Compatibility
- ROS 2 Humble not available via apt, requires source build
- Workaround: Use ROS 2 Iron or Foxy (with updated examples)

### Older GPU Support
- RTX 1080/1070: May work but performance limited
- Workaround: Reduce simulation complexity, use simplified models

### RAM Constraints
- 16GB systems: Possible but may require swap configuration
- Workaround: Use lighter simulation models, reduce concurrent processes

## Academic Compliance

All examples and code snippets in this textbook:
1. Compatible with Ubuntu 22.04 LTS
2. Verified with ROS 2 Humble Hawksbill
3. Tested on minimum hardware specifications
4. Include alternative approaches for resource-constrained environments