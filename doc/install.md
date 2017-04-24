# Installation of the ROS/IOP Bridge

## Install JAUS Toolset
Get [JAUS Toolset](http://jaustoolset.org/) (JTS), e.g. from Github:

    git clone https://github.com/dvmartin999/jaustoolset.git

You need **[Oracle Java 7 JDK](https://help.ubuntu.com/community/Java)** (does not work with java 8!) to compile the JAUS Tool Set. The easiest way to install Java 7 is listed here:

    sudo apt-get install python-software-properties
    sudo add-apt-repository ppa:webupd8team/java
    sudo apt-get update
    sudo apt-get install oracle-java7-installer
    sudo apt-get install oracle-java7-set-default

Install  **scons** and **ant**:

    sudo apt-get install scons ant

Now compile JTS:

    1. goto jaustoolset/GUI/
    2. ant bindmxGraph
    3. ant bindJSIDLPlus
    4. ant bind
    5. ant compile-promela
    6. ant compile
    7. ant schema-export
    8. export JTS_COMMON_PATH=/home/XXX/jaustoolset/GUI/templates/Common
    9. goto jaustoolset/nodeManager
    10. scons

>If some errors occur regarding missing *pthread* and *timer* the *libpthread* and *librt* have to be included. This can be done by replacing LIBS=[] by LIBS=['-lpthread', '-lrt'] in   *GUI/templates/Common/SConstruct*

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

## Build the sources

Go to the root folder of your ROS workspace and type:

    catkin_make
    
>In case of "permission denied" errors you have to mark the \*.py files in *jaus\_builder\_fkie/cmake/* as runnable
>You can also use `catkin build` instead of `catkin_make`, if you have [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) installed
