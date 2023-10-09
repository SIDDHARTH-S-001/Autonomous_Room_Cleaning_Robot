# auto_clean
Follow these instructions to work with the robot.

<h3>Installation</h3>

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
<h3>SLAM</h3>
<br>If successful, launch the gazebo file with lockstep mode set to "True" (to sync sensor update rate and physics engine update rate)</br>
<br>This will open gazebo and RVIz with the robot spawned.</br>
<br>![image](https://github.com/SIDDHARTH-S-001/auto_clean/assets/73553742/0becfd97-cb37-4248-91a3-3b3b2b98df72)</br
<br>You can implementing SLAM by launch Gmapping.</br>

```
roslaunch auto_clean_description gazebo.launch lockstep:=true
roslaunch auto_clean_description gmapping,launch
```
<br>In RVIZ click the Add button (bottom left) the add a map.</br
<br>Dont forget to change the fixed frame to map (default will be base_link)</br>
<br>Launch Teleop twist keyboard in new terminal tab to drive the robot manually</br>

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
<br>You should see something like this (a 2D occypancy grid map)</br>
<br>![image](https://github.com/SIDDHARTH-S-001/auto_clean/assets/73553742/7ea6d3a3-d3fd-4e93-938c-32db87dda8ef)</br>
<br>After you're satisfied, you can safe the map using map saver, open a new terminal tab and execute</br>

```
rosrun map_server map_saver -f mymap

```

<h3>Navigation</h3>
<br>You can start Autonomous Navigation by executing the following lines.</br>

```
roslaunch auto_clean_description gazebo.launch lockstep:=true
roslaunch auto_clean_description navigation.launch
```
<br>NavFn is set as the default global planner, which can be changed as per your preference. You'll find a few options in the scripts/autonomous folder</br>
<br>Remember to add the necessary widgets for visualization in RVIZ.</br>
<br>You can set your goak location using the 2D Nav tool.</br>
<br>If everything works, you should see the robot moving towards the goal location on its own like in the picture below</br>
<br><img width="701" alt="image" src="https://github.com/SIDDHARTH-S-001/auto_clean/assets/73553742/ebde4c50-6bca-4df9-9944-7bf05df638ea"></br>

<br>In case you prefer navigating the robot into a pre-saved map, then execute this instead</br>

```
roslaunch auto_clean_description map_navigation.launch
```









