
This repository lets your ROS software communicate with IOP services.

>currently only the core services and a set of selected components for driving is implemented. Pacakges for manipulating are currently depricated and are under construction.


# Installation of the ROS/IOP Bridge

## Install dependencies for JAUS Toolset
ROS/IOP Bridge uses JAUS Toolset (JTS) to generate basic C++ code from JSIDL definitions of JAUS. You find the JAUS Toolset desciption [here](http://jaustoolset.org/). Currently a fort of JTS is used.
JTS will be downloaded from github.com and build while you build the ROS/IOP Bridge sources. For successful build JTS needs follow external software:

Install OpenJKD:

    sudo apt-get install default-jdk -y

>If you prefer Oracle Java, see [here](doc/install_oracle_java.md)

Install  **scons** and **ant**:

    sudo apt-get install scons ant -y

## Set up your ROS environment

Make sure, that your catin workspace is set up correctly:

- Using [catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- Using [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace)

For easier download of bridge packages install **[wstool](http://wiki.ros.org/wstool)**:

    sudo apt-get install python-wstool -y

If you want to use a manipulator, you will have to install moveit:

    sudo apt-get install ros-kinetic-moveit

>For other ROS version replace kinetic by your ROS version


----
## Download the ROS/IOP-Bridge packages

The sourcecode of the ROS/IOP-Bridge is splitted into different Git repositories. Depending on your configuration you need a different set of packages. An overview of existing packages you find [here](doc/other_packages.md)

For download the ROS/IOP-Bridge sources we use the [wstool](http://wiki.ros.org/wstool):
  > If you do not already have an *.rosinstall* go into you ROS workspace and call
```
cd catkin_ws/
wstool init src
```

Merge the iop.rosinstall file and fetch code.
```
wstool merge -t src https://raw.githubusercontent.com/fkie/iop_core/master/iop.rosinstall
wstool update -t src
```

## Build the sources

Go to the root folder of your ROS workspace and type:

    catkin build

>In case of "permission denied" errors you have to mark the \*.py files in *jaus\_builder\_fkie/cmake/* as runnable
>You can also use `catkin build` instead of `catkin_make`, if you have [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) installed

>If some errors occur while JTS build regarding missing *pthread* and *timer* the *libpthread* and *librt* have to be included. This can be done by replacing LIBS=[] by LIBS=['-lpthread', '-lrt'] in   *jaustoolset/build/jaustoolset/GUI/templates/Common/SConstruct*

## Additional Information

For convenient usage of ROS environment use the `node_manager` of `multimaster_fkie`. You can install it from  [https://github.com/fkie/multimaster_fkie](https://github.com/fkie/multimaster_fkie) using
```
wstool merge -t src https://raw.githubusercontent.com/fkie/multimaster_fkie/master/multimaser_fkie.rosinstall
wstool update -t src
```

On each host you run IOP components you need to start the JTS-`nodeManager`:

```bash rosrun jaustoolset jaus_node_manager.sh start ```

>To exit the script type `bash rosrun jaustoolset jaus_node_manager.sh stop` in a new terminal window (`CTRL+C` won't work).
>See example below how to integrate the JTS-`nodeManager`into launch-file.


## Example

The **iop_cfg_sim_stage_fkie** package contains working configuration files specified to run with a stage simulator.

You can find the description to this example [here](https://github.com/fkie/iop_cfg_sim_stage_fkie/blob/master/README.md)


----

## [How it works - an overview](doc/how_it_works.md)
## [IOP packages - an overview](doc/other_packages.md)
## [IOP core packages](doc/iop_core_packages.md)
## [Builder-package](doc/builder_package.md)
