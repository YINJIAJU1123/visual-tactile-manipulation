# Grasp Robotic Arm Workflow

以下是运行流程的详细说明，所有命令均需要在 Conda 虚拟环境中运行。

```bash
conda activate grasp
```

## Profile 1

### Step 1: Connect and Start the Robotic Arm

1. 启动 ROS 主节点：

    ```bash
    roscore
    ```

2. 进入工作空间目录：

    ```bash
    cd arm_ws/
    ```

3. 分别在三个终端中运行以下命令：

    - **启动 UR5 机械臂驱动**：

      ```bash
      roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=10.10.10.1
      ```

      等待输出信息：

      ```
      [INFO] [<timestamp>]: Controller Spawner: Waiting for service controller_manager/load_controller
      ```

    - **启动 MoveIt! 规划与执行**：

      ```bash
      roslaunch ur5_moveit_config moveit_planning_execution.launch limited:=true
      ```

      输出提示：

      ```
      You can start planning now!
      ```

    - **启动 RViz 可视化**：

      ```bash
      roslaunch ur5_moveit_config moveit_rviz.launch config:=true
      ```

4. 在新打开的 RViz 中，添加以下模块：
    - **Motion Planning**
    - **Robot Model**

   然后将固定框架（Fixed Frame）从 `map` 改为 `base_link`。

5. 将机械臂移动到“home”位置。

### Step 2: Open the Camera

1. 打开新终端，进入 `graspnet-baseline` 目录：

    ```bash
    cd graspnet-baseline/
    ```

2. 启动摄像头程序：

    ```bash
    python shendu.py
    ```

3. 程序运行几秒后，按下 `S` 键，等待深度图对焦。

### Step 3: Calculate the Grasping Pose

1. 打开新终端，进入 `graspnet-baseline` 目录：

    ```bash
    cd graspnet-baseline/
    ```

2. 运行以下命令以解除环境变量的限制：

    ```bash
    unset LD_LIBRARY_PATH
    ```

3. 启动抓取姿态计算程序：

    ```bash
    ./run.sh
    ```

4. 关闭第一个接收到的图像，抓取姿态将自动生成并显示。

### Step 4: Drive the Robotic Arm

1. 确保当前目录为 `graspnet-baseline`：

    ```bash
    cd graspnet-baseline/
    ```

2. 运行机械臂控制程序：

    ```bash
    python ur5_cartesian.py
    ```

3. 操作完成。

---

### 注意事项
- 确保每一步骤在正确的终端中运行。
- 若遇到问题，请检查相关配置文件与环境变量。
- 所有代码假设已正确配置 ROS 和 MoveIt! 环境。

