# mirobot_order_delivery

ROS2 Humble laptop-side package for the Mirobot + camera + YOLO worker.

This cleaned version **does not include the FastAPI/server code**. The Raspberry Pi server remains separate. This package only does:

1. subscribe `/robot/cmd`
2. detect requested red/green/blue cubes with YOLO
3. convert pixel center to Mirobot base XY using the dynamic calibration convention
4. pick cube with vacuum pump
5. move to the box position and release
6. repeat by order count
7. publish `/robot/done`

## Default domain layout

```text
Raspberry Pi server: ROS_DOMAIN_ID=18
Laptop Mirobot/YOLO: ROS_DOMAIN_ID=30
```

`config/mirobot_domain_bridge.yaml` bridges:

```text
18 -> 30: /robot/cmd, /robot/stop
30 -> 18: /robot/done, /robot/status
```

## Install

On the laptop/Mirobot PC:

```bash
sudo apt update
sudo apt install ros-humble-domain-bridge

cd ~/Mirobot_ros2/src
unzip /path/to/mirobot_order_delivery_domain_bridge.zip

source ~/yolov5_env/bin/activate
source /opt/ros/humble/setup.bash

cd ~/Mirobot_ros2
colcon build --packages-select mirobot_order_delivery
source install/setup.bash
```

## Run laptop robot worker + domain bridge

```bash
source ~/yolov5_env/bin/activate
source /opt/ros/humble/setup.bash
source ~/Mirobot_ros2/install/setup.bash

ros2 launch mirobot_order_delivery robot_with_bridge.launch.py server_domain:=18 robot_domain:=30
```

This starts both:

- `domain_bridge`
- `mirobot_order_delivery_node` on robot domain 30

## Run Raspberry Pi server

No package code change is required on the Raspberry Pi side. Run your existing server on domain 18:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
python3 main_auto_delivery.py
```

or with uvicorn if that is how you run it:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
uvicorn main_auto_delivery:app --host 0.0.0.0 --port 8000
```

## Test without the app

Keep `robot_with_bridge.launch.py` running on the laptop. In another terminal, publish a test order into the server domain:

```bash
source ~/yolov5_env/bin/activate
source /opt/ros/humble/setup.bash
source ~/Mirobot_ros2/install/setup.bash
export ROS_DOMAIN_ID=18

ros2 run mirobot_order_delivery simulate_order --id 1 --red 1 --green 1 --blue 1
```

The bridge forwards `/robot/cmd` from domain 18 to domain 30, the laptop node performs the task, then `/robot/done` is bridged back to domain 18.

## Main tuning file

Edit:

```bash
~/Mirobot_ros2/src/mirobot_order_delivery/config/order_delivery.yaml
```

Important fields:

```yaml
serial.port: "/dev/ttyUSB0"
camera.id: 4
yolo.repo: "/home/kjy/yolov5"
yolo.weight: "/home/kjy/yolov5/runs/train/exp/weights/best.onnx"

poses.observe_xyz: [140.0, 0.0, 150.0]
poses.box_xyz: [200.0, 0.0, 70.0]
poses.pick_z: 55.0
poses.travel_z: 150.0
poses.box_approach_z: 150.0
poses.pick_y_offset_mm: 15.0

calibration.fallback_origin_u: 640.0
calibration.fallback_origin_v: 360.0
calibration.fallback_origin_x: 140.0
calibration.fallback_origin_y: 0.0
calibration.fallback_mm_per_px: 0.50
calibration.dynamic_rotation_deg: 0.0
```

## Dynamic calibration convention

Default mapping uses your direction result:

```text
camera pixel: +u = right, +v = down
robot XY:     +X = up,    +Y = left

dX = -dv * scale
dY = -du * scale
```

Then it applies optional `calibration.dynamic_rotation_deg` in robot XY if the camera axes are slightly rotated.

## Static calibration optional

If you later want to use `static_board_pose.npz`, copy it into:

```bash
~/Mirobot_ros2/src/mirobot_order_delivery/data/static_board_pose.npz
```

and set:

```yaml
calibration.mode: "static"
calibration.use_static: true
```

## Run without bridge

For local testing in only one domain:

```bash
source ~/yolov5_env/bin/activate
source /opt/ros/humble/setup.bash
source ~/Mirobot_ros2/install/setup.bash

ros2 launch mirobot_order_delivery robot_only.launch.py robot_domain:=30
```

Then publish `/robot/cmd` from another terminal with `ROS_DOMAIN_ID=30`.
