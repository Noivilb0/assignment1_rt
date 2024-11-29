# Assignment1_rt

This package implements a ROS-based application with two nodes:
- **UI Node**: Spawns turtle2 on run, Allows user to control turtles in the turtlesim environment, chossing the speeds of the linear and angular velocities and the direction is dictated using the sign of the input value.
- **Distance Node**: Monitors distance between turtles and enforces constraints, up to this point i still achieve consistent stopping everytime, buti'm working on understanding the reason why.
