# optimal-control

Automatic motion planning is one of the most significant challenges facing autonomous mobile robotics. The goal is to be able to specify a task in a high-level language and have the robot automatically compile this specification into a set of low-level motion primitives, or feedback controllers, to accomplish the mission. 

The typical task is to find a path for a wheeled robot, such as a Roomba or a self-driving car, from one configuration to another.

One of the most significant advantages of the usage of an autonomous mobile robot in an ordinary context is its autonomy, which allows it to scan an area for obstacles on its own. This also applies to environments that may change over time, such as a group of people entering the working area and posing obstacles to the robot. For example, instead of a complete path, the robot is given the starting and ending points and is instructed to create its path while scanning and avoiding anything that gets in its way.

When traveling from point A to point B, it is not required to take a specific route. This means that autonomous mobile robots are more adaptable to new tasks. They can be reprogrammed more easily for new routes or layouts.

Referring to the present work, a relatively simple trajectory planning task has been considered.
A mobile robotic platform, thanks to an optimal control algorithm, must reach some viapoints (with relative intermediate configurations) placed in the work environment, at the end performing a classic operation of returning to the base (the starting point). All this is done by optimizing the path according to some constraints.

The one used is a strategy that generates the control action by solving an optimization problem in a finite time interval, starting from a known initial condition, and managing, according to the current state, the state and the control input autonomously based on the parameters chosen in a specific cost index, reaching the target position and minimizing the final positioning error at the same time.

A problem of this type is the same common to domestic robots of daily use such as vacuum cleaners, industrial mobile robots such as those used for warehouse storage, helping customers build a safer working environment and more cost-effective productivity, and finally, robots used for individual mobility in the context of simple transport or other autonomous tasks such as the monitoring and security of an area.
In this last framework, the aforementioned optimal control algorithm has been developed.
