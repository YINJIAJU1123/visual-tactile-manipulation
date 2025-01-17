# Grasp Robotic Arm Workflow

The following is a detailed workflow. All commands need to be executed within a Conda virtual environment.

```bash
conda activate grasp
```

## Profile 1

### Step 1: Connect and Start the Robotic Arm

1. Start the ROS master node:

    ```bash
    roscore
    ```

2. Navigate to the workspace directory:

    ```bash
    cd arm_ws/
    ```

3. Run the following commands in three separate terminals:

    - **Launch the UR5 robot driver**:

      ```bash
      roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=10.10.10.1
      ```

      Wait for the output message:

      ```
      [INFO] [<timestamp>]: Controller Spawner: Waiting for service controller_manager/load_controller
      ```

    - **Launch MoveIt! planning and execution**:

      ```bash
      roslaunch ur5_moveit_config moveit_planning_execution.launch limited:=true
      ```

      You should see the prompt:

      ```
      You can start planning now!
      ```

    - **Launch RViz visualization**:

      ```bash
      roslaunch ur5_moveit_config moveit_rviz.launch config:=true
      ```

4. In the newly opened RViz window, add the following modules:
    - **Motion Planning**
    - **Robot Model**

   Then change the Fixed Frame from `map` to `base_link`.

5. Move the robotic arm to the "home" position.

### Step 2: Open the Camera

1. Open a new terminal and navigate to the `graspnet-baseline` directory:

    ```bash
    cd graspnet-baseline/
    ```

2. Start the camera program:

    ```bash
    python shendu.py
    ```

3. After a few seconds, press the `S` key and wait for the depth map to focus.

### Step 3: Calculate the Grasping Pose

1. Open a new terminal and navigate to the `graspnet-baseline` directory:

    ```bash
    cd graspnet-baseline/
    ```

2. Run the following command to unset the environment variable:

    ```bash
    unset LD_LIBRARY_PATH
    ```

3. Start the grasp pose calculation program:

    ```bash
    ./run.sh
    ```

4. Close the first received image, and the grasp pose for the robotic arm will appear.

### Step 4: Drive the Robotic Arm

1. Ensure you are in the `graspnet-baseline` directory:

    ```bash
    cd graspnet-baseline/
    ```

2. Run the robotic arm control program:

    ```bash
    python ur5_cartesian.py
    ```

3. The operation is complete.

---

### Notes
- Ensure that each step is executed in the correct terminal.
- If issues arise, check the relevant configuration files and environment variables.
- The instructions assume that ROS and MoveIt! environments are correctly configured.
