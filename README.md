# Assignment1_rt  (Mohamed ElHefnawy)(7312537)

This package implements a ROS-based application with two nodes (As assigned in Our research track course) :
- **UI Node**: Spawns turtle2 on running the node, Allows user to control turtles in the turtlesim environment, chossing the speeds of the linear(x) and angular(z) velocities and the direction is dictated using the sign of the input value.
- **Distance Node**: Monitors distance between turtles and enforces constraints, achieves consistent stopping everytime, Halts the movements of both turtles whenever a threshold parameter is crossed.
