# emotion_tom

```sh
mkdir ~/emotion_tom
git clone https://github.com/GiovanniFalcone/emotion_tom.git src
catkin_make
source ./devel/setup.bash
chmod u+x src/hri/app/src/app.py
chmod u+x src/hri/emotion/src/emotion_node.py
chmod u+x src/hri/robot/src/manager_node.py
chmod u+x src/hri/robot/src/perception.py
```

## How to play

```sh
roslaunch robot usb_cam-test.launch
roslaunch app app.launch condition:=0
roslaunch robot controller.launch emotion_condition:=true
```

