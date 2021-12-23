
# SLAM

This SLAM implementation leverages the localization module `laser_scan_matcher`. The placing of the cones is performed by mapping the cones' relative positions with respect to the car using its absolute position `pose_stamped` generated by `laser_scan_matcher`.

  

## Usage

  

<!--

The slam toolbox package can be downloaded at the following link [here](https://github.com/SteveMacenski/slam_toolbox)

  

It's important to note that the provided toolbox builds the map using a `sensor_msgs::LaserScan` but the LIDAR outputs data of type `sensor_msgs::PointCloud`.

  

To address this iussue, we transform the point cloud into laser scan using the [`pointclod_to_laserscan package](http://wiki.ros.org/pointcloud_to_laserscan)

  

The slam toolbox listens for `LaserScan` messages on the topic specified in `slam_toolbox/config` in the param `scan_topic`.

-->

  

Hereby follows a concise guide on how to assembly the various components:

```console

foo@bar:~$ git clone https://github.com/unipi-smartapp-2021/SLAM

foo@bar:~$ cd SLAM

foo@bar:~$ git submodule update --init --recursive

foo@bar:~$ sudo apt-get install libgsl0-dev

```

  

If `catkin_make` fails due to missing `csm` package, install it:

```console

foo@bar:~$ cd src

foo@bar:~$ git clone https://github.com/AndreaCensi/csm

```

  

Overwrite the following files:

```console

foo@bar:~$ cp utils/pointcloud_to_laserscan_nodelet.cpp src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_nodelet.cpp

foo@bar:~$ cp utils/sample_node.launch src/pointcloud_to_laserscan/launch/sample_node.launch

foo@bar:~$ cp utils/laser_scan_matcher.cpp src/scan_tools/laser_scan_matcher/src/laser_scan_matcher.cpp

```

  

Try and pray that everything builds:

```console

foo@bar:~$ catkin_make

```

  

## Run it

**Important** You should follow this exact same order in order to succesfully launch the SLAM:

```console

foo@bar:~$ roscore

foo@bar:~$ rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node

foo@bar:~$ rosrun cone_mapping cone_mapping.py

foo@bar:~$ rosparam set use_sim_time true

```

Then launch the bag, localization and set the simulation clock:

```console

foo@bar:~$ rosbag play <bag> --clock

foo@bar:~$ rosrun laser_scan_matcher laser_scan_matcher_node

```

Prints the output topics:

```console

foo@bar:~$ rostopic echo /pose_stamped

foo@bar:~$ rostopic echo /cone_right

foo@bar:~$ rostopic echo /cone_left

```

  

## Results

  

If you want to plot the cones, you must create a bag recording the topic `/cone_left /cone_right /pose_stamped` and then launch `utils/visualize_cones.py`. Example:

  

![](imgs/track.png)

  

After having cleaning up the noise points:

  

![](imgs/track_2.png)

  

In order to apply the identity function that allows us to remove noise points displayed in the figure above, we have just applied the following idea:

If the new cone is at distance d from an already discovered one, and d < distance_threshold,

the new cone is not inserted in the list of cones.

  

However in this way, the position of the cone is never updated, but in real scenarios the first detection of the cone can be very noisy and must be updated during the run.

So, we should apply some avaraging on the position of the cone here.

In the TODO list there also some other interesting possible improvements.

  

We also applied an avarage on the colors detected for a specific cone and the final result is:

  

![](imgs/track_3.png)

  

You can run the *cone_drawing* node to visualize at runtime the published cones. Just run

```console

foo@bar:~$ rosrun cone_mapping cone_drawing.py

```
before playing the rosbag.
  

# TODO

- [ ] Make launchfile

- [ ] Use directly Pointercloud instead of converting to Laserscan

- [ ] Test on the simulator

-  [X] Avaraging color detections

- [ ] Avaraging points of detected cones

  

<!--

## slam-toolbox

  

**IMPORTANT** before doing anything change the branch to `noetic-devel`

  

Install dependencies with `rosdep install -q -y -r --from-paths src --ignore-src`

  

Install `apt install ros-noetic-slam-toolbox` if required.

  
  

## pointcloud-to-laserscan

  

**IMPORTANT** before doing anything change the branch to `lunar-devel`

  

Notice that `geometry2` is required to build this package. -->
