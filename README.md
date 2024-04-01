# ROS2 Unity VR Point Cloud Visualization

## Overview
This guide details the visualization of point cloud data from ROS2 Foxy in Unity VR. The visualization pipeline is divided into two main parts: setting up ROS to publish point cloud data, and setting up Unity to visualize this data in VR.

## Part 1: ROS Setup

### Prerequisites:
- Windows Subsystem for Linux (WSL2) with Ubuntu 20.04
- ROS2 Foxy
- `rosbridge_suite` for the WebSocket connection between ROS and Unity

### Installing `rosbridge_suite`:
To install `rosbridge_suite`, use the following command:

```bash
sudo apt-get install ros-<rosdistro>-rosbridge-server
```

### Using a Bag File:
To play a pre-recorded bag file, use:

```bash
ros2 bag play file.db3
```

### Checking Available Topics:
To list all the published topics, use:

```bash
ros2 topic list
```

### Echoing Point Cloud Data:

To echo the point cloud data being published on /sensing/lidar/top/rectified/pointcloud:

```bash
ros2 topic echo /sensing/lidar/top/rectified/pointcloud
```

### Launching rosbridge_server:
To start the WebSocket server on port 9090:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Part 2: Unity Setup

### Scripts and Shaders:
- enableOpenGL.cs: Loads OpenGL and enables the GL_VERTEX_PROGRAM_POINT_SIZE.
- PointCloudRenderer.cs: Handles arrays for color and position (using Vector3) for each point.
- PointCloudShader.shader: Contains a _PointSize property and should not be renamed.

### Unity VR Setup Steps:
- Right-click and select XR > Device_based > XR Origin(VR).
- Add enableOpenGL.cs script to the Main Camera.
- Attach PointCloudSubscriber.cs to the RosConnector object (assuming ROSConnector is added). Configure the point cloud topic name.
- Create a pointCloudRenderer for PointCloudRenderer.cs.
- Drag RosConnector to the Subscriber field and set the Point Size in PointCloudRenderer.cs.
- Link the camera offset to the Offset in PointCloudRenderer.cs to position the point clouds correctly.
