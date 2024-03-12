# LANES ARE INSANE
## Colby Beach and Hope Crisafi

### Basic Overview

Our project's main goal is for autonomous driving around a set track. When our Turtlebot reaches an object in its, it should be able to stop itself, wait for the object to move, and if it doesn't move then it should go around the object and back into the lanes.


### How to run

The most important package needed is the [Turtlebot3 Autorace 2020 Package](https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020). This **NEEDS** to be installed *correctly* in order for our code to run. [Install Turtorial Here](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/).

##### Calibrating The Turtlebot:

The first step to ensure our code runs correctly is to calibrate our Turtlebot's camera with the lanes. There are two types of calibration, intrinsic and extrinsic, but we found that intrinsic calibration is not really necessary to make everything run well, so we will skip that for now (it also is a biotch)

Steps for extrinsic calibration: 

- Install this package and autorace_2020 (duh)
- Place our turtlebot within the track with the yellow lane to the left of the camera, and white lane to the right. 
- roscore (duh)
- roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch (on the turtlebot ssh)
- roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
- roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration
- rqt

Now, within rqt running, we want to set an imageview window with the view of
- /camera/image_extrinsic_calib/compressed

And run:

 - rosrun rqt_reconfigure rqt_reconfigure

 With rqt_reconfigure, we want to set the red box to fit within the lane, with the top two corners hitting the edges. 

 [See this tutorial again for more tips](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/).


##### Lane detection Calibration
Once we have extrinsicly calibrated our turtlebot's camera with the lanes, we can now calibrate the lane detection code with our lanes. 

**THIS STEP IS VITAL AND OUR CODE WILL SUCK IF THIS IS NOT DONE CORRECTLY**

Steps:


- Place our turtlebot within the track with the yellow lane to the left of the camera, and white lane to the right. 
- roscore (duh)
- roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch (on the turtlebot ssh)
- roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
- roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
- roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration
- rqt
- rosrun rqt_reconfigure rqt_reconfigure

Once we have all of these running, we want to have three imageview windows open in rqt, showing off: 

1. /detect/image_yellow_lane_marker/compressed
2. /detect/image_lane/compressed
3. /detect/image_white_lane_marker/compressed


With these shown, we want to adjust the parameters in detect_lane key in rqt_reconfigure until the white and yellow lanes are clearly marked in white in the rqt image windows, and in the /detect/image_lane/compressed window we should see three lines outlining the lanes and the middle, with a green highlight within the lane, showing that the code now understands where the lane is. Adjusting these parameters I found is truly trial and error, and it depends on the Turtlebot, the lighting, and even just what day you try it. 

[See this tutorial again for more tips](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/).

##### Running our code:

Once we have calibrated everything completely, we can finally make it do its thing! This is where our code deviates from the tutorial I keep linking, so this README will now be the best resource

Steps: 
- Run all of the code from the last step (except the rqt windows unless you want them open to see how it is performing).
- SSH again into the Turtlebot in another terminal and run roslaunch turtlebot3_bringup turtlebot3_robot.launch
- Run our lane_and_avoid.py code from wherever you installed the package (example - rosrun avoid_obstacles lane_and_avoid.py).

Everything should now work!!! Lets goooooo (we hope)

### Some small notes:

- The turtlebot obstacle detection can be a bit finnicky, so we found the best way to test everything is with a piece of paper that can directly be seen by the lidar. 
- When we run our code, in theory we should be running the detect_lane code in its mode:=action instead of mode:=calibrate. For whatever reason we could not get this to work however, so we keep it in calibrate against the advice of the tutorial. *It works so much better*
- Have fun! If anything goes wrong, just kill our lane_and_avoid.py code, reset the turtlebot in the track, and try again :)
