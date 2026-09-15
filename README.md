# TAP02 – Industrial Robotics with ROS 2 and MoveIt 2

## FANUC M-10iA Pick-and-Place Simulation

This repository contains the ROS 2 workspace developed for the Industrial
Robotics course at Universidad EIA.

The project implements a simulated pick-and-place cell for the FANUC M-10iA
robot using:

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2
- RViz 2
- OMPL
- ros2_control with mock hardware
- Python 3
- MATLAB for DH-model validation

The MoveIt configuration was created specifically for this project from the
robot description.

> This project runs with simulated mock hardware. It must not be used directly
> to control a real industrial robot.

---

## 1. Project objectives

The project covers the following tasks:

1. Create a custom MoveIt configuration for the FANUC M-10iA.
2. Validate the HOME transformation using TF2 and a DH model in MATLAB.
3. Calculate PICK and PLACE inverse-kinematics solutions with MoveIt.
4. Validate the PICK and PLACE configurations with the DH model.
5. Create a collision scene with a workpiece, supports, and an obstacle.
6. Compare RRTConnect and RRTstar for obstacle avoidance.
7. Generate cubic and quintic fine-approach profiles.
8. Execute a complete collision-free pick-and-place cycle in RViz.
9. Attach the workpiece to `tool0` during transportation.
10. Detach the workpiece at the PLACE position.

The analytical Jacobian comparison is planned as the next project stage.

---

## 2. Robot and reference frames

| Property | Value |
|---|---|
| Robot | FANUC M-10iA |
| Planning group | `manipulator` |
| Base frame | `base_link` |
| End-effector frame | `tool0` |
| Number of joints | 6 revolute joints |
| IK solver | KDL |
| Main planner | RRTConnect |
| Controller | `fanuc_arm_controller` |
| Hardware | `mock_components/GenericSystem` |

The controlled joints are:

```text
joint_1
joint_2
joint_3
joint_4
joint_5
joint_6
```

---

## 3. Main Cartesian poses

The positions are expressed in metres in the `base_link` frame.

| Pose | Position `[x, y, z]` | Quaternion `[x, y, z, w]` |
|---|---|---|
| PRE_PICK | `[0.75, -0.30, 0.95]` | `[1.0, 0.0, 0.0, 0.0]` |
| PICK | `[0.75, -0.30, 0.85]` | `[1.0, 0.0, 0.0, 0.0]` |
| PRE_PLACE | `[0.75, 0.30, 0.95]` | `[1.0, 0.0, 0.0, 0.0]` |
| PLACE | `[0.75, 0.30, 0.85]` | `[1.0, 0.0, 0.0, 0.0]` |

The 0.10 m fine-approach distance is a design choice for this project; it was
not imposed by the assignment.

---

## 4. Repository structure

```text
TAP02_RI_ET06/
├── matlab/
│   └── validation_home.m
├── src/
│   ├── fanuc_description/
│   ├── et06_m10ia_moveit_config/
│   └── et06_pick_place/
│       ├── config/
│       │   └── key_poses.yaml
│       ├── results/
│       └── scripts/
│           ├── setup_scene.py
│           ├── benchmark_planners.py
│           ├── summarize_benchmark.py
│           ├── generate_approach_profiles.py
│           ├── compute_pick_approach.py
│           ├── compute_place_approach.py
│           ├── retime_approach.py
│           ├── validate_retimed_trajectory.py
│           ├── execute_pick_approach.py
│           ├── attach_workpiece.py
│           ├── lift_workpiece.py
│           ├── transfer_workpiece.py
│           ├── execute_place_approach.py
│           ├── detach_workpiece.py
│           └── run_pick_and_place.py
├── README.md
└── REPRISE.md
```

The `build/`, `install/`, and `log/` directories are generated locally by
`colcon` and do not need to be downloaded from Git.

---

## 5. System requirements

The recommended environment is:

```text
Ubuntu 24.04 LTS
ROS 2 Jazzy
MoveIt 2 for ROS 2 Jazzy
Python 3
Git
colcon
rosdep
```

This repository is intended for Linux. MATLAB is optional and is only required
for the DH validation and profile plots.

---

## 6. Install ROS 2 and MoveIt 2

Install ROS 2 Jazzy before continuing. After the ROS installation, install the
main development tools:

```bash
sudo apt update

sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml
```

Initialize `rosdep` if this has not already been done:

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` reports that initialization was already completed, this
message can be ignored.

Install MoveIt and the required ROS packages:

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-setup-assistant \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-controller-manager \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  ros-jazzy-rviz2
```

---

## 7. Clone the repository

Open a terminal and run:

```bash
cd ~

git clone https://github.com/antoinemnier/TAP02_RI_ET06.git

cd ~/TAP02_RI_ET06
```

If the repository is already present, update it with:

```bash
cd ~/TAP02_RI_ET06

git pull origin main
```

---

## 8. Install workspace dependencies

From the workspace root:

```bash
cd ~/TAP02_RI_ET06

source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -r \
  -y
```

---

## 9. Build the workspace

Compile all packages:

```bash
cd ~/TAP02_RI_ET06

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
```

After a successful build, source the workspace:

```bash
source ~/TAP02_RI_ET06/install/setup.bash
```

This command must be executed in every new terminal used for the project.

Optional: add the setup commands to Bash:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/TAP02_RI_ET06/install/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

---

## 10. Run the complete simulation

Two terminals are required.

### Terminal 1 — Start MoveIt and RViz

```bash
cd ~/TAP02_RI_ET06

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch et06_m10ia_moveit_config demo.launch.py
```

Wait until:

- RViz is open;
- the FANUC robot is visible;
- MoveIt has finished loading;
- the robot is initially at HOME.

Do not close this terminal.

### Terminal 2 — Run the complete cycle

Open a second terminal:

```bash
cd ~/TAP02_RI_ET06

source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 src/et06_pick_place/scripts/run_pick_and_place.py
```

The program displays the project requirements and asks:

```text
Type START to run the sequence:
```

Enter exactly:

```text
START
```

The program then performs:

1. planning-scene creation;
2. HOME to PRE_PICK transfer;
3. quintic fine approach to PICK;
4. workpiece attachment;
5. vertical lifting;
6. transfer to PRE_PLACE;
7. quintic fine approach to PLACE;
8. workpiece detachment.

A successful run ends with:

```text
CYCLE COMPLETED
The workpiece was placed and detached.
```

---

## 11. Important execution conditions

Before starting the complete cycle:

- `demo.launch.py` must be running;
- the robot must be at HOME;
- no other motion script may be running;
- no other script may modify the planning scene;
- the first terminal must remain open;
- the workpiece must initially be a world object and not attached to `tool0`.

If a planning attempt fails because OMPL is stochastic, restart the cycle from
HOME and try again.

Do not run two motion scripts at the same time.

---

## 12. Run individual project stages

Normally, use `run_pick_and_place.py`. The following scripts are useful for
testing individual stages.

First prepare each terminal:

```bash
cd ~/TAP02_RI_ET06

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Create and validate the planning scene

```bash
python3 src/et06_pick_place/scripts/setup_scene.py
```

Expected result:

```text
HOME         : VALID
PRE_PICK     : VALID
PICK         : VALID
PRE_PLACE    : VALID
PLACE        : VALID
```

### Execute HOME → PRE_PICK → PICK

```bash
python3 src/et06_pick_place/scripts/execute_pick_approach.py
```

### Attach the workpiece

```bash
python3 src/et06_pick_place/scripts/attach_workpiece.py
```

### Lift the workpiece

```bash
python3 src/et06_pick_place/scripts/lift_workpiece.py
```

### Transfer the workpiece to PRE_PLACE

```bash
python3 src/et06_pick_place/scripts/transfer_workpiece.py
```

### Execute PRE_PLACE → PLACE

```bash
python3 src/et06_pick_place/scripts/execute_place_approach.py
```

### Detach the workpiece

```bash
python3 src/et06_pick_place/scripts/detach_workpiece.py
```

The individual movement scripts may request explicit confirmation before
execution.

---

## 13. Planner comparison

The project compares two OMPL planners:

- RRTConnect
- RRTstar

Make sure MoveIt and RViz are running, then execute:

```bash
cd ~/TAP02_RI_ET06

source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 src/et06_pick_place/scripts/setup_scene.py

python3 src/et06_pick_place/scripts/benchmark_planners.py
```

The benchmark performs several planning trials without executing the robot.

The original project experiment produced:

| Planner | Success rate | Main observation |
|---|---:|---|
| RRTConnect | 10/10 | Fast and reliable |
| RRTstar | 1/10 | Shorter successful path, but low reliability |

RRTConnect was selected for the complete cycle.

Because OMPL uses randomized sampling, exact results can vary between runs.

### Summarize an existing benchmark

Benchmark results are stored in timestamped folders under:

```text
src/et06_pick_place/results/
```

Run:

```bash
python3 src/et06_pick_place/scripts/summarize_benchmark.py \
  src/et06_pick_place/results/RESULT_FOLDER
```

Replace `RESULT_FOLDER` with the actual folder name.

For example:

```bash
python3 src/et06_pick_place/scripts/summarize_benchmark.py \
  src/et06_pick_place/results/20260908_153443_847604
```

The summary includes:

- success rate;
- planning time;
- joint-space path length;
- trajectory duration;
- RMS joint acceleration.

---

## 14. Generate cubic and quintic profiles

The assignment requires the temporal profiles to be generated in Python.

Run:

```bash
cd ~/TAP02_RI_ET06

python3 src/et06_pick_place/scripts/generate_approach_profiles.py
```

The profile files are written under:

```text
src/et06_pick_place/results/approach_profiles/
```

Expected files include:

```text
cubic_red.csv
quintic_red.csv
cubic_blue.csv
quintic_blue.csv
```

The red limits are interpreted as:

```text
Maximum Cartesian velocity:     0.200 m/s
Maximum Cartesian acceleration: 0.300 m/s²
```

The blue limits are interpreted as:

```text
Maximum Cartesian velocity:     0.100 m/s
Maximum Cartesian acceleration: 0.020 m/s²
```

These values are treated as upper limits.

---

## 15. Compute Cartesian paths

The following commands require MoveIt to be running.

### PICK approach

```bash
python3 src/et06_pick_place/scripts/compute_pick_approach.py
```

A complete path should report:

```text
Result code: 1
Computed fraction: 1.00000000
```

### PLACE approach

The workpiece must be attached and the robot must be at PRE_PLACE before this
command is used:

```bash
python3 src/et06_pick_place/scripts/compute_place_approach.py
```

A complete path should also report a fraction of `1.0`.

---

## 16. Time reparameterization

### PICK approach using the red limits

```bash
python3 src/et06_pick_place/scripts/retime_approach.py \
  pick red
```

### PLACE approach using the blue limits

```bash
python3 src/et06_pick_place/scripts/retime_approach.py \
  place blue
```

The reparameterized joint trajectories are stored in:

```text
src/et06_pick_place/results/retimed/
```

The complete cycle uses the selected quintic profiles.

---

## 17. Validate reparameterized trajectories

MoveIt and the project planning scene must be running.

### Validate the PICK approach

```bash
python3 src/et06_pick_place/scripts/validate_retimed_trajectory.py \
  pick red
```

### Validate the PLACE approach

The workpiece must be attached to `tool0`:

```bash
python3 src/et06_pick_place/scripts/validate_retimed_trajectory.py \
  place blue
```

The original trajectories were validated over 2001 sampled states.

A successful validation reports:

```text
VALID: 2001/2001 points.
```

No robot motion is commanded by the validation script.

---

## 18. MATLAB DH validation

MATLAB is used to validate:

- HOME: DH against TF2;
- PICK: DH against MoveIt forward kinematics;
- PLACE: DH against MoveIt forward kinematics.

The main script is:

```text
matlab/validation_home.m
```

Open MATLAB, change the current folder to:

```text
TAP02_RI_ET06/matlab
```

and run:

```matlab
validation_home
```

Expected position errors are close to numerical zero:

| Configuration | Approximate position error |
|---|---:|
| HOME | `3.51e-13 m` |
| PICK | `1.11e-16 m` |
| PLACE | `1.92e-16 m` |

The script can also display the cubic and quintic profiles if the generated CSV
files are placed in the same MATLAB folder.

For MATLAB Online, upload these files next to the MATLAB script:

```text
cubic_red.csv
quintic_red.csv
cubic_blue.csv
quintic_blue.csv
```

---

## 19. Useful ROS 2 checks

### List active controllers

```bash
ros2 control list_controllers
```

Expected controllers:

```text
joint_state_broadcaster
fanuc_arm_controller
```

Both should be `active`.

### Read the current joint state

```bash
ros2 topic echo /joint_states --once
```

### Read the current tool transformation

```bash
ros2 run tf2_ros tf2_echo base_link tool0 -p 12
```

Press `Ctrl+C` to stop `tf2_echo`.

### Check IK and FK services

```bash
ros2 service list -t | grep -E 'compute_ik|compute_fk'
```

Expected services:

```text
/compute_ik [moveit_msgs/srv/GetPositionIK]
/compute_fk [moveit_msgs/srv/GetPositionFK]
```

### Check available planning interfaces

```bash
ros2 service call \
  /query_planner_interface \
  moveit_msgs/srv/QueryPlannerInterfaces \
  "{}"
```

---

## 20. Main results

### DH validation

The DH model agrees numerically with TF2 and MoveIt for HOME, PICK, and PLACE.

### Planner comparison

RRTConnect was selected because it was substantially faster and more reliable
than RRTstar under the selected five-second planning budget.

### Cartesian approaches

Both fine Cartesian paths were completely generated:

```text
Computed fraction: 1.0
```

### Selected interpolation profile

The quintic profile was selected because it provides:

- zero endpoint velocity;
- zero endpoint acceleration;
- smoother transitions at the beginning and end of the fine approach.

### Complete cycle

The complete simulated cycle was successfully executed in RViz:

```text
HOME
→ PRE_PICK
→ PICK
→ attach workpiece
→ lift workpiece
→ PRE_PLACE
→ PLACE
→ detach workpiece
```

---

## 21. Known limitations

- The project uses mock hardware, not a real FANUC controller.
- It does not simulate complete robot dynamics.
- The gripper is represented by logical attachment to `tool0`.
- The 0.10 m approach distance is a project design choice.
- The red and blue values are interpreted as maximum constraints.
- The current analytical Jacobian comparison is not yet included.
- OMPL results can vary because sampling-based planning is stochastic.
- The benchmark smoothness indicator uses RMS joint acceleration; joint jerk
  could be included in a more detailed study.

---

## 22. Troubleshooting

### Package not found

```text
Package 'et06_m10ia_moveit_config' not found
```

Run:

```bash
cd ~/TAP02_RI_ET06

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### MoveIt services are unavailable

If a script displays:

```text
waiting for service to become available
```

make sure this command is running in another terminal:

```bash
ros2 launch et06_m10ia_moveit_config demo.launch.py
```

### Robot is not at the expected position

The movement scripts verify their initial state. Restart the simulation so the
robot returns to HOME, then run the complete cycle again.

### Planning returns an error

A planning error may occur because OMPL is randomized. Check that:

- the planning scene is loaded;
- the robot is in the correct initial state;
- the obstacle has not been changed;
- no other motion is running.

Then restart from HOME and try again.

### Python module or import error

Make sure both environments were sourced:

```bash
source /opt/ros/jazzy/setup.bash
source ~/TAP02_RI_ET06/install/setup.bash
```

### Profile CSV files are missing

Generate them with:

```bash
python3 \
  src/et06_pick_place/scripts/generate_approach_profiles.py
```

---

## 23. Safety notice

This repository is intended for educational simulation only.

Before adapting any trajectory to a real industrial robot, it would be
necessary to review:

- the real robot model and payload;
- tool and work-object calibration;
- controller communication;
- speed and acceleration limits;
- singularities;
- safety zones;
- emergency-stop procedures;
- physical collision risks.

---

## 24. Authors and course

Course:

```text
Industrial Robotics
Mechatronics Engineering
Universidad EIA
```

Assignment:

```text
Taller – Cinemática y Planeación de Movimiento con ROS2 / MoveIt2
Team ET06
FANUC M-10iA
```

Authors:

```text
Antoine Meunier
Diego Oxman
```
