# Humanoid Robot Simulation

This repository branch contains a simulation environment for a humanoid robot ErgoCub, built using Gazebo Classic, ROS 2 Galactic, and YARP. The goal is to enable testing of sensor data, SLAM algorithms, and control in a reproducible Docker container setup.

## 🧩 Repository Contents

- Docker image with pre-installed Gazebo, ROS 2, and YARP
- ROS 2 launch files
- YARP-to-ROS 2 bridge utilities
- Gazebo worlds and models
- `.desktop` shortcuts for launching inside Docker

---

## 🚀 How to Launch the Simulation

### Option 1: Using Docker Compose (recommended)

Use the following command to launch:

```bash
docker compose up
```


The container will launch a KasmVNC server for GUI apps (e.g., Gazebo) avaible on https://container_ip:6901 and the following local directories are mounted:

| Local Directory        | Container Path                        | Purpose                                |
|------------------------|----------------------------------------|----------------------------------------|
| `simulation_container/gazebo_worlds/`           | `/opt/gazebo_worlds`                                                            | User-defined Gazebo worlds             |
| `gazebo_models/`                                | `/usr/share/gazebo/models`                                                     | Custom Gazebo models                   |
| `simulation_container/ergocub_modified/`        | `/opt/robotology-superbuild/build-base/install/share/ergoCub/robots/ergocub_modified` | Modified ErgoCub robot model          |
| `simulation_container/yarp-ros2-ergocub-bridging/` | `/opt/yarp-ros2-ergocub-bridging`                                              | YARP–ROS 2 bridging config/scripts     |
| `simulation_container/python/`                  | `/opt/code/`                                                                    | Python control/bridge scripts          |


### Option 2: Manual Docker Run

If you prefer not to use Compose, manually define volumes, parameters and pass usb devices:

```bash
docker run -it --rm \
  --name humanoid_robot-simulation \
  --network bridge \
  -p 6901:6901 \
  -p 11345:11345 \
  -e VNC_PW=password \
  -e TZ=Europe/Prague \
  -e PUID=1000 \
  -e PGID=1000 \
  -v /mnt/user/appdata/humanoid-robot-simulation/simulation_container/python:/opt/code \
  -v /mnt/user/appdata/humanoid-robot-simulation/simulation_container/yarp-ros2-ergocub-bridging:/opt/yarp-ros2-ergocub-bridging \
  -v /mnt/user/appdata/humanoid-robot-simulation/simulation_container/gazebo_models:/usr/share/gazebo/models \
  -v /mnt/user/appdata/humanoid-robot-simulation/simulation_container/gazebo_worlds:/opt/gazebo_worlds \
  -v /mnt/user/appdata/humanoid-robot-simulation/simulation_container/ergocub_modified:/opt/robotology-superbuild/build-base/install/share/ergoCub/robots/ergocub_modified \
  -v /mnt/user/appdata/humanoid-robot-simulation/simulation_container/ergocub_slam_bridge:/opt/ros_ws/src/ergocub_slam_bridge \
  --device=/dev/input/js0 \
  vlepic/humanoid-robot-simulation:gazeboclassic-ROS2


```

> **Note:** If you omit volume mappings, your local models, bridges, and world files will not be accessible inside the container.
**Note:** If you omit passing a joystick - you will not be able to run the walking controller and SLAM packages.

---

## 🧠 Docker Image Content

This image `vlepic/humanoid-robot-simulation:gazeboclassic-ROS2` includes:

- Ubuntu with ROS 2 Galactic
- Gazebo Classic
- YARP libraries
- Python YARP–ROS 2 bridge utilities
- SLAM and simulation tools

---

## 🧭 Running the Simulation Workflow

After launching the container (either via Docker Compose or `docker run`), follow these steps:

1. **Access the Web Interface:**
   - Open your web browser and go to: `http://container_ip:6901`
   - Login credentials:
     - **Username:** `kasm_user`
     - **Password:** `password`

2. **Launch Core Components Using Desktop Shortcuts:**
   - Inside the remote desktop:
     - Start **YARP Server** using the `.desktop` launcher.
     - Start **Gazebo Server** using the appropriate `.desktop` launcher (which includes correct arguments).

3. **Start Walking Modules:**
   - Use the `Walking Modules (Combined)` launcher.
   - ⚠️ **Note:** The joystick must be passed to the container (see below), or the walking modules will not start.

4. **Joystick Setup:**
   - Ensure your joystick device is mapped when starting the container, e.g.:
     ```bash
     --device=/dev/input/js0
     ```

5. **Walking Control:**
   - After walking modules load, press the **"Prepare Walking"** button on the joystick.
   - Wait for the console message: `Robot prepared`
   - Then press the "Start Walking" button on the joystick. You should see periodic telemetry output in the console, indicating the controller is active.

6. **Start SLAM + RViz:**
   - Use the `.desktop` launcher labeled `SLAM with YARP`.
   - This will launch SLAM Toolbox and RViz.
   - You can now walk around using the joystick and observe mapping and localization in RViz.

---

## 📝 License

This project is licensed under the MIT License.

---

## 📫 Contact

Author: [Václav Lepic](https://github.com/VLepic)
