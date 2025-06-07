# odom_mover

`odom_mover` is a ROS package designed to control robot movement based on odometry data.

## Features

- Subscribes to a specified odometry topic (e.g., `/odometry/filtered` topic)
- Processes odometry data and publishes movement commands (e.g., to the `/cmd_vel` topic)
- Allows customization of movement goals or behaviors

## Build and Usage

1. **Install dependencies** as required by your ROS environment.
2. **Create a workspace** and clone this package into the `src` directory.
3. **Build the workspace** using:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```
## Example

```bash
rosrun odom_mover odom_mover3
```