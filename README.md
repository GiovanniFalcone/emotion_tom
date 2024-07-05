# emotion_tom

```sh
mkdir ~/emotion_tom
catkin_make
source ./devel/setup.bash
chmod u+x src/hri/app/src/app.py
chmod u+x src/hri/emotion/src/emotion_node.py
chmod u+x src/hri/robot/src/interaction.py
chmod u+x src/hri/robot/src/perception.py
```

## How to play

```sh
roslaunch usb_cam usb_cam-test.launch _video_device:=/dev/video0
roslaunch app app.launch condition:=0
roslaunch robot controller.launch emotion_condition:=true
```
