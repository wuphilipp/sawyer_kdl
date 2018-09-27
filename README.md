# sawyer_kdl
Example usage of PyKDL using the sawyer robot

## Dependencies

### Robot saywer description ros package
https://github.com/RethinkRobotics/sawyer_robot/tree/master/sawyer_description

### Sawyer common ros package
https://github.com/RethinkRobotics/intera_common

### PyKDL
```pip install PyKDL```

## Usage
Launch the saywer robot in RViz
```roslaunch sawyer_kdl sawyer_kdl_test.launch```

The script will print out the forward kinematics frame and the jacobian for the current set of joint positions.
The joint positions can be changed by dragging the bars in the ```joint_state_publisher``` gui.
