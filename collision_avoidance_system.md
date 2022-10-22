# Robot Collision Avoidance System 1.0

### Assumptions
1. Implementors are responsible for detecting/predicting collisions between robots.
2. Robots ignore possible collisions with a robot that is behind them. Basically, don't tailgate other robots,
you are guilty 100% of the time when you rear end someone. And if you brake check somebody, John will put your robot in time out.

### Collisions
Collisions are defined as two robots being on two paths that intersect **AND** within 500mm of each other.
The 500mm "bubble" will allow the robots to navigate around each other with approximately 158mm/6in of clearance.


### Collision Avoidance Method
Once a collision is detected, the robots should begin driving clock-wise in an arc with radius of 250mm. They
should drive 180 degrees(assuming no additional collision), and then continue on their path. 
