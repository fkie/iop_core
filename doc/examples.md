For convenient usage of ROS environment use the `node_manager` of `multimaster_fkie`. You can install it from ROS apt repository or from  [https://github.com/fkie/multimaster_fkie](https://github.com/fkie/multimaster_fkie) using

    git clone https://github.com/fkie/multimaster_fkie

On each host you run IOP components you need to start the JTS-`nodeManager`:
```bash
rosrun fkie_iop_builder jaus_node_manager.sh start
```

## `fkie_iop_cfg_sim_stage`
This package contains working configuration files specified to run with a stage simulator.

You can find the description to this example on https://github.com/fkie/iop_examples/blob/master/fkie_iop_cfg_sim_stage/README.md
