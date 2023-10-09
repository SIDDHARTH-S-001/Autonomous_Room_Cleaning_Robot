# auto_clean
Follow these instructions to work with the robot.

<h3>Instructions</h3>

```
cd {your_catkin_workspace}/src
git clone https://github.com/SIDDHARTH-S-001/auto_clean.git</br>
cd ..
catkin_make
```

  
<h3>Check if the package exists in your workspace</h3>

```
 rospack find auto_clean_description
```
  
<h3>If successful, launch the gazebo file with lockstep mode set to "True" (to sync sensor update rate and physics engine update rate)</h3>
<br>This will open gazebo and RVIS along with the gmapping node launched.</br>

```
roslaunch auto_clean_description gazebo.launch lockstep:=true
```


<br>In RVIZ click the Add button (bottom left) the add a map.</br
<br>Dont forget to change the fixed frame to map (default will be base_link)</br>

<br>Launch Teleop twist keyboard in new terminal tab to drive the robot manually</br>

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


