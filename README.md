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
roslaunch robot usb_cam-test.launch
roslaunch app app.launch condition:=0
roslaunch robot controller.launch emotion_condition:=true
```

## Slide
https://drive.google.com/file/d/147Aj5_smTKgTa9YJSGVJN1RKqXD2_QNm/view?usp=sharing
