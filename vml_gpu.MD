## Creating __virtualenv__
* Go to the working directory (in our case `~/Dev/dqn`)
* Create a vitual environment to install ROS ,Sage, and Tensorflow in:
  - `$ virtualenv .`
* Activate the environmentL
  - On the centOS which has __csh__ as default shell: `$ source bin/activate.csh`

## Installing ROS:
* `$ module laod LANG/PYTHON/2.7.10-GCC492`
* Make sure you are in the virtual environment and it's active
* `$ pip install -U rosdep rosinstall_generator wstool rosinstall`
* `$ pip install --upgrade setuptools`
* `$ pip install catkin-tools`
* change the /etc folder for ros installation
  - in `lib/python2.7/site-packages/rosdep2/sources_list.py`
  - and in `lib/python2.7/site-packages/rospkg/environment.py`
  - also changing `ROS_ETC_DIR` might work ?!
* `$ rosdep init`
* `$ rosdep update`
* `$ mkdir -p /dqn_ws/src
* `$ cd dqn_ws`
* `$ catkin init`
* `$ rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall`
* `$ wstool init -j8 src kinetic-ros_comm-wet.rosinstall`
* You need to install __console_bridge__ separately:
  - `$ module load COMPILER/GNU/6.2.0`
  - `$ git clone git://github.com/ros/console_bridge.git`
  - `$ cd console_bridge`
  - `$ cmake . -DCMAKE_C_COMPILER=/rcg/software/Linux/RHEL/6/x86_64/COMPILER/GNU/4.9.2/bin/gcc -DCMAKE_CXX_COMPILER=/rcg/software/Linux/RHEL/6/x86_64/COMPILER/GNU/4.9.2/bin/g++  -DCMAKE_INSTALL_PREFIX=/grad/2/smohaime/Dev/dqn/`
  - `$ make`
  - `$ make install`
* rospack needs __tinyxml__:
  - `$ git clone https://github.com/leethomason/tinyxml2.git`
  - `$ cd tinyxml2`
  - `$ cmake . -DCMAKE_C_COMPILER=/rcg/software/Linux/RHEL/6/x86_64/COMPILER/GNU/4.9.2/bin/gcc -DCMAKE_CXX_COMPILER=/rcg/software/Linux/RHEL/6/x86_64/COMPILER/GNU/4.9.2/bin/g++  -DCMAKE_INSTALL_PREFIX=/grad/2/smohaime/Dev/dqn/`
  - `$ make`
  - `$ make install`
* Invoke catkin_make_isolated from downloaded packages:
  - `$ ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release`
* 
