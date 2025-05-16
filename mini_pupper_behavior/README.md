
## Introduction 
This package is based on the dance package.

This implementation is made for controlling easily the behaviors of mini pupper by calling high-level functions. So the mini pupper can be asked to turn, move forward or achieve more complex behaviors such as "yes", "no" or show pre-coded emotion behaviors like being sad, happy, playful.

The movements/behaviors that the the robot will execute are entered in the client file. Please modify to change the behaviors if needed.

The main interface is described in the "mini_pupper_interfaces" package. For more details, please refer to the "BehaviorCommand.srv".

## Quick Start

### Mini Pupper

```sh
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

### PC (Or Mini Pupper)
```sh
# terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_behavior behavior.launch.py
```

```sh
# terminal 2
source ~/ros2_ws/install/setup.bash
ros2 run mini_pupper_behavior behavior_client
```
### How to modify

This package includes the following Python scripts in the mini_pupper_behavior/mini_pupper_behavior folder:
- behavior_client.py: The client reads and sends behavior commands. 
- behavior_server.py: The server receives behavior commands and executes them. You can add more behavior functions in behavior_server.py.
- pose_controller.py: A pose controller for Mini Pupper. You don't need to modify this.