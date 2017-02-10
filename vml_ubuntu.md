## Creating __virtualenv__
* Go to the working directory (in our case `~/Dev/ubuntu_sys/dqn`)
* Create a vitual environment to install ROS ,Stage, and Tensorflow in:
  - `$ virtualenv .`
* Activate the environmentL
  - On the centOS which has __csh__ as default shell: `$ source bin/activate.csh`

## Installing ROS:
* `$ module laod LANG/PYTHON/2.7.10-GCC492`
* Make sure you are in the virtual environment and it's active
* `$ pip install -U rosdep rosinstall_generator wstool rosinstall`
* `$ pip install --upgrade setuptools`
* `$ pip install catkin-tools`
* Activate the virtualenv again
* Change the /etc folder for ros installation
  - in `lib/python2.7/site-packages/rosdep2/sources_list.py`
  - and in `lib/python2.7/site-packages/rospkg/environment.py`
  - also changing `ROS_ETC_DIR` might work ?!
* `$ rosdep init`
* `$ rosdep update`
* `$ mkdir -p /dqn_ws/src
* `$ cd dqn_ws`
* `$ rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall`
* `$ wstool init -j8 src kinetic-ros_comm-wet.rosinstall`
* `$ pip install empy`
* You need to install __console_bridge__ separately:
  - `$ module load COMPILER/GNU/6.2.0`
  - `$ git clone git://github.com/ros/console_bridge.git`
  - `$ cd console_bridge`
  - `$ cmake . -DCMAKE_C_COMPILER=/rcg/software/Linux/RHEL/6/x86_64/COMPILER/GNU/4.9.2/bin/gcc -DCMAKE_CXX_COMPILER=/rcg/software/Linux/RHEL/6/x86_64/COMPILER/GNU/4.9.2/bin/g++  -DCMAKE_INSTALL_PREFIX=/grad/2/smohaime/Dev/dqn/`
  - `$ make`
  - `$ make install`
* _rospack_ needs __tinyxml__:
  - I ended up copying it from another ubntu Xenial distribution.
  - Link to downlad `TBA`
* _roslz4_ needs __lz4__:
  - `$ apt-get download liblz4-dev`
  - `$ ar x liblz4-dev_0.0~r131-2ubuntu2_amd64.deb`
  - `$ apt-get download liblz4-1`
  - `$ ax r liblz4-1_0.0~r131-2ubuntu2_amd64.deb`
  - Copy the files in `usr` folder that was just sxtracted to the correct folders.
* Invoke catkin_make_isolated from downloaded packages:
  - `$ ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release`
* After fist attempt, I don't know why _genmsg_ package is not being found by _std\_msgs_ package, had to add the path to all currently build packages to `PYTHONPATH`:
  - `$ export PYTHONPATH=$PYTHONPATH:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/catkin/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/genmsg/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/gencpp/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/geneus/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/genlisp/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/gennodejs/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/genpy/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rosboost_cfg/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rosclean/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/roscreate/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rosgraph/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rosmake/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rosmaster/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/roslib/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rosparam/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rospy/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rosservice/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/roslaunch/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rosunit/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/roslz4/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/rostest/lib/python2.7/dist-packages:/home/smohaime/Dev/ubuntu_sys/dqn/ws/devel_isolated/std_msgs/lib/python2.7/dist-packages`
* And Revoke the catkin_make_isolated:
  - `$ ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release`
* And build Succeeds !!!!

=====================================================================

##Compilng the __stage__ and __stage_ros__ packages:

* Generate the rosinstall file:
  - `$ rosinstall_generator stage stage_ros --rosdistro kinetic --deps | wstool merge -t src -`
  - `$ wstool update -t src -j8
  - Remove testing from __tf__ & __tf2__'s _CMakeList.txt_ file.
  - `$ ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release`
  
