# auto_clean
Follow these instructions to work with the robot.

<h3>Instructions</h3>
```cd {your_catkin_workspace}/src```

git clone https://github.com/SIDDHARTH-S-001/auto_clean.git</br>
cd ..
```
catkin_make
```

  
### 3) Check if the package exists in your workspace
  rospack find auto_clean_description
  
### 4) Then launch the gazebo file with lockstep function (to sync sensor update rate and physics engine update rate)
  roslaunch auto_clean_description gazebo.launch lockstep:=true

This will open gazebo and RVIS along with the gmapping node launched.
### 5) In RVIZ click the Add button (bottom left) the add a map.
   Dont forget to change the fixed frame to map (default will be base_link)

### 6) Launch Teleop twist keyboard using 
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py 



