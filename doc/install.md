# Installation of the ROS/IOP Bridge

## Install dependencies for JAUS Toolset
ROS/IOP Bridge uses JAUS Toolset (JTS) to generate basic C++ code from JSIDL definitions of JAUS. You find the JAUS Toolset desciption [here](http://jaustoolset.org/).
JTS will be downloaded from github.com and build while you build the ROS/IOP Bridge sources. For successful build JTS needs follow external software:

**Java JDK**. Either **OpenJDK 1.8** or **[Oracle Java 7 JDK](https://help.ubuntu.com/community/Java)** (does not work with java 8!) to compile the JAUS Tool Set.
Install OpenJKD:

    sudo apt-get install default-jdk

The easiest way to install Java 7 is listed here:

    sudo apt-get install python-software-properties
    sudo add-apt-repository ppa:webupd8team/java
    sudo apt-get update
    sudo apt-get install oracle-java7-installer
    sudo apt-get install oracle-java7-set-default

Install  **scons** and **ant**:

    sudo apt-get install scons ant

## Set up your ROS environment

Make sure, that your catin workspace is set up correctly:

- Using [catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- Using [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace)

For newer ROS versions (since Kinetic) you will have to define the CPATH environment variable manually. Add the following line to your `~/.bashrc` file and make sure to insert the correct path to your workspace:

    export CPATH=<path-to-ros-iop-bridge-workspace>/devel/include

If you want to use a manipulator, you will have to install moveit:

    sudo apt-get install ros-kinetic-moveit

>For other ROS version replace kinetic by your ROS version


----
## Download the ROS/IOP-Bridge packages

The sourcecode of the ROS/IOP-Bridge is splitted into different Git repositories. Depending on your configuration you need a different set of packages.

The repositories are:
- **iop_core**: core pakages of the ROS/IOP-Bridge needed to build the bridge
	- git clone https://github.com/fkie/iop_core

- Repositories with packages implementing the SAE JAUS Services
	- **core**: _urn.jaus.jss.core_ - services
		- git clone https://github.com/fkie/iop_jaus_core
	- **mobility**: _urn.jaus.jss.mobility_ - services
		- git clone https://github.com/fkie/iop_jaus_mobility
	- **sensing**: _urn.jaus.jss.environmentSensing - services
		- git clone https://github.com/fkie/iop_jaus_sensing

- Repositories with packages implementing the **clients** for SEA JAUS Services
	- **mobility_clients**: clients for _urn.jaus.jss.mobility_ - services
		- git clone https://github.com/fkie/iop_jaus_mobility_clients
	- **sensing_clients**: clients for _urn.jaus.jss.environmentSensing - services
		- git clone https://github.com/fkie/iop_jaus_sensing_clients

- IOP services
	- **iop_platform**: the platform services including all services to create a platform
		- git clone https://github.com/fkie/iop_platform
	- **iop_sensing**: sensor services
		- git clone https://github.com/fkie/iop_sensing

- **iop_gui**: ROS GUI for IOP services
	- git clone https://github.com/fkie/iop_gui

- **iop_cfg_sim_stage_fkie**: Example configuration
	- git clone https://github.com/fkie/iop_cfg_sim_stage_fkie


To install the complete ROS/IOP-Bridge go into your ROS workspace and clone these repository:

	git clone https://github.com/fkie/iop_core
	git clone https://github.com/fkie/iop_jaus_core
	git clone https://github.com/fkie/iop_jaus_mobility
	git clone https://github.com/fkie/iop_jaus_sensing
	git clone https://github.com/fkie/iop_jaus_mobility_clients
	git clone https://github.com/fkie/iop_jaus_sensing_clients
	git clone https://github.com/fkie/iop_platform
	git clone https://github.com/fkie/iop_sensing
	git clone https://github.com/fkie/iop_gui
	git clone https://github.com/fkie/iop_cfg_sim_stage_fkie

You can also use the [wstool](http://wiki.ros.org/wstool):
  > If you do not already have an *.rosinstall* go into you ROS workspace and call
```
cd catkin_ws/
wstool init src
```

Merge the iop.rosinstall file and fetch code.
```
wstool merge -t src https://raw.githubusercontent.com/fkie/iop_core_fkie/master/iop.rosinstall
wstool update -t src
```

## Build the sources

Go to the root folder of your ROS workspace and type:

    catkin_make

>In case of "permission denied" errors you have to mark the \*.py files in *jaus\_builder\_fkie/cmake/* as runnable
>You can also use `catkin build` instead of `catkin_make`, if you have [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) installed

>If some errors occur while JTS build regarding missing *pthread* and *timer* the *libpthread* and *librt* have to be included. This can be done by replacing LIBS=[] by LIBS=['-lpthread', '-lrt'] in   *jaustoolset/build/jaustoolset/GUI/templates/Common/SConstruct*

