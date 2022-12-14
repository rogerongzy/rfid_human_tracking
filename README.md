# rfid_human_tracking

## Prerequisites
- [Ubuntu 20.04](https://github.com)
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)



## Running Command
The ros node of the whole robot's bringing up:
```Cmd
roslaunch main_run navi.launch
```

The visual interface of rviz:
```Cmd
rosrun rviz rviz
```

The ros node of realsense d435 camera with depth aligned:
```Cmd
roslaunch realsense2_camera rs_d435_camera_with_model.launch align_depth:=true
```

The ros node of darknet_ros and corresponding yolo_v3 weight:
```Cmd
roslaunch darknet_ros yolo_v3.launch
```

The controller of move_base library:
```Cmd
python3 controller.py
```

The ros node of rfid collecting:
```Cmd
rosrun rfid rfid 169.254.1.1
```

The ros node of rfid predicting AOA:
```Cmd
python3 rfid_AOA.py
```