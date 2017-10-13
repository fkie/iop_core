## How ROS/IOP-Bridge works - an overview

The ROS/IOP-Bridge consists of a lot of independent components. This components represent in case one the IOP components:
By **iop_component_fkie** it is now possible to configure the set of the services included into one component. All services of the ROS/IOP Bridge are implemented as a plugin. You can configure to use all services in one component or in a lot of independent components.

![](images/iop-bridge-case1.png)

For the IOP-OCU it looks like IOP components, but in the background they use the ROS data.
In the second case the ROS/IOP-Bridge components are used on the OCU side to process the JAUS data and make it available for ROS-GUI, like rqt:

![](images/iop-bridge-case2.png)

### Component structure
As an example we look at the PrimitiveDriver service plugin. This component is located in [iop_mobility](https://github.com/fkie/iop_jaus_mobility) and implemnents the translation of `SetWrenchEffort` and `ReportWrenchEffort` to ROS messages:

![](images/iop-bridge-component.png)

To reduce the implementation effort of included JAUS services like `AccessControl`, `Discovery` and other we implemented these as libraries wich are included by the component. Look at `package.xml` and `CMakeLists.txt` of `iop_primitive_driver_fkie` package to see how they can be included.

### Components on the OCU side
The ROS/IOP-Bridge components on the OCU side consume the JAUS data, e.g. of the PrimitiveDriver. This components are located in packages beginning with `iop_client_` prefix and include the client services for `AccessControl`, `Discovery` and other. Additionally, we created `iop_ocu_slavelib_fkie` and `ocu_rqt_access_control_fkie` packages to control the independent client components simultaneously. This is useful to release or get access for a whole robot for example.

![](images/iop-bridge-controllib-layer.png)
