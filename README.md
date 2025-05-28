# emotion_tom
This projects use **ROS Noetic**.

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
roslaunch app app.launch id:=<id> condition:=<1/.../6>
roslaunch robot controller.launch emotion_condition:=<true/false> open_webcam:=<true/false>
```

*Check "HowToPlay.pdf" for more informations*.
