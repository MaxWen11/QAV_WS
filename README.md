
# UADL-QAV250: Uncertainty-Aware Dynamics Learning & Control Framework

![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)
![OS](https://img.shields.io/badge/OS-Ubuntu%2020.04-orange.svg)
![C++](https://img.shields.io/badge/C++-17-blue.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

## 📖 Overview
This repository contains the official implementation of the **Uncertainty-Aware Dynamics Learning and Control** framework. Designed to bridge the "sim-to-real" gap for quadrotors facing complex coupled disturbances (e.g., aerodynamic drag and payload swing), this ROS-based workspace integrates:

1. **Offline Structural Priors**: Neural network generators trained via GAN to capture nominal system dynamics.
2. **Online Bayesian Refinement**: Real-time sliding-window Gaussian Process (GP) regression to analytically decouple residual nonlinearities and quantify model uncertainties.
3. **Robust Tube-based MPC (RTMPC)**: An uncertainty-aware optimal controller that dynamically scales safety boundaries (tubes) and scheduling gains based on the GP's posterior variance.

The system is deployed on a custom QAV250 quadrotor running PX4, utilizing UWB and ToF sensors for centimeter-level indoor state estimation.

---## 🛠️ Prerequisites & Dependencies

The code is developed and tested under **Ubuntu 20.04** and **ROS 1 Noetic**.

### 1. ROS Noetic & MAVROS
Install standard ROS Noetic and MAVROS to communicate with the PX4 flight controller:

```bash
sudo apt update
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

### 2. qpOASES (For RTMPC Optimization)
The Robust Tube-based MPC requires the qpOASES solver. Compile and install it globally:

```bash
cd ~
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES && mkdir build && cd build
cmake .. -DCMAKE_CXX_FLAGS="-fPIC"
make -j$(nproc)
sudo make install
```

### 3. LibTorch (For Real-time GAN Prior Inference)
Download the C++11 ABI CPU version from the PyTorch website and extract it to your home directory:

```bash
cd ~
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip
```

## 🚀 Installation & Build
Clone the Workspace:

```bash
mkdir -p ~/QAV_WS/src && cd ~/QAV_WS/src
git clone <YOUR-GITHUB-REPO-URL> .
```

Build the Workspace:
You must provide the path to LibTorch during compilation to successfully build the neural network inference module.

```bash
cd ~/QAV_WS
catkin_make -DCMAKE_PREFIX_PATH="~/libtorch;/opt/ros/noetic"
```

Source Environment:

```bash
source /opt/ros/noetic/setup.bash
source ~/QAV_WS/devel/setup.bash
```

## 🚁 How to Run (Flight Test)
To safely run the full closed-loop system, we recommend using a multi-pane terminal (e.g., tmux or terminator). Ensure the drone is powered on and connected via USB or Telemetry (/dev/ttyACM0).

### Terminal 1: MAVROS Connection
Establish communication with the PX4 flight controller:

> **Note**: Copy `utils/extras.txt` to the Pixhawk SD card (specifically `/etc/extras.txt`) to increase the odometry publishing frequency to ~100Hz.

```bash
source /opt/ros/noetic/setup.bash
source ~/QAV_WS/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600 --wait
```

### Terminal 2: Localization (UWB & ToF)
Launch the Nooploop nlink_parser for LinkTrack (UWB) and ToFSense, followed by the bridge node for state estimation. A sequential startup is used to prevent serial port conflicts:

```bash
source /opt/ros/noetic/setup.bash
source ~/QAV_WS/devel/setup.bash
# Run UWB, wait, run ToF, wait, then launch bridge
roslaunch --wait nlink_parser linktrack.launch &
sleep 5
roslaunch --wait nlink_parser tofsense.launch &
sleep 3
rosrun uwb_bridge uwb_bridge_node
```

### Terminal 3: Core Controller
Launch the Uncertainty-Aware Controller (px4ctrl), which executes the Online GP and RTMPC algorithms.(Make sure your TorchScript models .pt are placed in the correct execution directory).

```bash
source /opt/ros/noetic/setup.bash
source ~/QAV_WS/devel/setup.bash
roslaunch px4ctrl run_ctrl.launch
```

### Terminal 4: Takeoff Command
Once all nodes are running smoothly and the localization data is stable, publish the takeoff command:

```bash
source /opt/ros/noetic/setup.bash
source ~/QAV_WS/devel/setup.bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```

⚠️ **SAFETY WARNING**: Always test the control commands without propellers first. Ensure your hardware Kill-Switch is mapped correctly on your RC transmitter before conducting real flight tests.

## 📊 Dataset: UADL-QAV250
This repository pairs with the UADL-QAV250 Dataset, which provides:

- **Offline Training Data**: Real-world flight dynamics data capturing exogenous wind fields and endogenous payload oscillations (Tug-of-war) for GAN prior training.
- **Online Evaluation Data**: Benchmark trajectories and tracking errors comparing this proposed framework against baseline LMPC and cold-start GPs.

## 📝 Acknowledgments
- Control architecture inspired by [Fast-Drone](https://github.com/ZJU-FAST-Lab/Fast-Drone-250).
- Hardware sensor drivers provided by [Nooploop](https://github.com/nooploop-dev/nlink_parser).
