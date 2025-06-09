# Humanoid Robot Simulation

This repository contains a simulation environment for a humanoid robot (e.g., ErgoCub), built using Gazebo Classic, ROS 2, and YARP. The goal is to enable testing of sensor data, SLAM algorithms, and control in a reproducible Docker container setup.

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


The container will launch with access to your X server for GUI apps (e.g., Gazebo). The following local directories are mounted:

| Local Directory        | Container Path                        | Purpose                                |
|------------------------|----------------------------------------|----------------------------------------|
| `gazebo_worlds/`       | `/opt/gazebo_worlds`                   | User-defined Gazebo worlds             |
| `models/`              | `/opt/models`                          | Custom Gazebo models                   |
| `yarp_bridge_pkg/`     | `/opt/code/yarp_bridge_pkg`            | YARP-ROS 2 bridge scripts              |
| `launch/`              | `/opt/code/yarp_bridge_pkg/launch`     | ROS 2 launch files                     |
| `/tmp/.X11-unix`       | `/tmp/.X11-unix`                       | Access to the X server for GUI         |
| `/dev`                 | `/dev`                                 | Access to hardware devices (optional)  |

### Option 2: Manual Docker Run

If you prefer not to use Compose, manually define volumes:

```bash
docker run -it --rm \
  --name humanoid_sim \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/gazebo_worlds:/opt/gazebo_worlds \
  -v $(pwd)/models:/opt/models \
  -v $(pwd)/yarp_bridge_pkg:/opt/code/yarp_bridge_pkg \
  -v $(pwd)/launch:/opt/code/yarp_bridge_pkg/launch \
  vlepic/humanoid-robot-simulation:gazeboclassic-ROS2
```

> **Note:** If you omit volume mappings, your local models, bridges, and world files will not be accessible inside the container.

---

## 🧠 Docker Image Content

The image `vlepic/humanoid-robot-simulation:gazeboclassic-ROS2` includes:

- Ubuntu with ROS 2 (e.g., Galactic or Humble)
- Gazebo Classic / Modern
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

## 🧩 Project Structure

```
humanoid-robot-simulation/
│
├── docker-compose.yml             # Docker Compose startup file
├── gazebo_worlds/                 # User-created worlds
├── models/                        # Custom models
├── yarp_bridge_pkg/              # YARP <-> ROS 2 bridge
├── launch/                        # ROS 2 launch files
├── docker/                        # .desktop launchers
└── README.md
```

---

## 📝 License

This project is licensed under the MIT License.

---

## 📫 Contact

Author: [Václav Lepic](https://github.com/VLepic)
