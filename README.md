# auto_clean

This project is a miniature room cleaning robot. The CAD Model has been designed in Autodesk Fusion 360 and later exported as URDF using the Fusion2URDF Plugin.

<h3>CAD</h3>
<br><img width="539" alt="image" src="https://github.com/SIDDHARTH-S-001/auto_clean/assets/73553742/83394303-32b2-4dac-a4ed-0b1c758231f0"></br>

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
<br>You can implement SLAM by launch Gmapping.</br>

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
<br>![image](https://github.com/SIDDHARTH-S-001/auto_clean/assets/73553742/ac4704ec-b388-449b-879e-e5ae321659ad)</br>
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

<h3>Coverage Planning</h3>
<br>Coming to the goal of the project, which is to clean every nook and corner of an indoor environment. For this we implement coverage planning</br>

```
roslaunch auto_clean_description full_coverage.launch
```
<br><img width="590" alt="image" src="https://github.com/SIDDHARTH-S-001/auto_clean/assets/73553742/a272ede7-f2db-4165-83f9-e42acfd43f18"></br>


<h2>References</h2>
<br>https://github.com/syuntoku14/fusion2urdf/tree/master</br>
<br>http://wiki.ros.org/navigation</br>
<br>https://github.com/nobleo/full_coverage_path_planner</br>
