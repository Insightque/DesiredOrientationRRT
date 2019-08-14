# DesiredOrientationRRT

# Instructions
- clone repository
1. clone this repository
2. `cd DesiredOrientationRRT`
---
- *build ompl source* 
3. `mkdir build`
4. `cd build`
5. `cmake ..`
6. `make`
---
- link ompl library & include files with ROS
7. `cd /opt/ros/<version>/lib/x86_64-linux-gnu`
8. `sudo ln -sf /path/to/DesiredOrientationRRT/build/lib  ./`
9. `sudo ln -sf /path/to/DesiredOrientationRRT/build/lib/*  ./`
10. `cd /opt/ros/<version>/include/`
11. `sudo ln -sf /path/to/DesiredOrientationRRT/src/ompl  ./`
---
- move DORRT package and do catkin_make
12. Move `ROS_packages/decision_maker` Package to your `catkin_ws`
12. do `catkin_make` in your `catkin_ws`
