# MQTT Topic Definitions

| Topic | Description |  Fields | Example | 
| :---: | :---: | :---: | :---: | 
| irobot_create3_swarm/pose/<robot_id> | The robot's absolute position and rotation in the iRobot Create3 coordinate system, relative to the origin (0,0), with all position measurements in meters. | **pose**(geometry_msgs/Pose), **twist**(geometry_msgs/Twist) and all sub-fields of these two objects. | _{"pose":{Pose object},"twist":{Twist object}}_ | 
| irobot_create3_swarm/robot_status/<robot_id> | A robot's status, from the set of robot states defined below. | **state**(uint8) | _{"state": PERAMBULATING}_|
| irobot_create3_swarm/robot_hazards/<robot_id> | The hazards detected by the robot. | **detections**(irobot_create3_msgs/HazardDetection[]) | _{"detections": [BUMP, CLIFF, STALL]}_ | 

### Topic Publishing Rates
The `irobot_create3_swarm/pose/[robot_id]` and `irobot_create3_swarm/robot_hazards/[robot_id]` should be published to as frequently as possible to keep all other robots up to date.
The robot status topic, however, is less important. We only need to know what it is when it changes, so publishing frequency can be flexible to what you want. It would be beneficial to
limit it somehow to reduce bandwidth consumption on the network.

### Robot States
The states below are `uint8` values.
| State | Description | 
| :---: | :---: |
| `OCCLUDED`=0 | Robot is not moving, can't decide where to go, can't see, or is lost. Set if the robot's covariance is high, but it is still moving towards a target. | 
| `PERAMBULATING`=1 | Robot may or may not be moving, but should be expected to change pose soon. Should only be set if the robot has a destination set and is moving purposefully towards it. | 
| `CONCUSSED`=2 | Robot is suffering from head trauma and does not know where it is. Send immediate medical assistance. Set if the robot's covariance is unusually high and it is not moving anymore. | 
| `DYING`=3 | Robot expects to go offline soon and since it does not know what happens after this life - being just a lowly robot - it wants the others to know where it died so they can go recover the body. |
| `R&R`4= | Robot is at home, sleeping or watching netflix. Do not disturb except in case of national emergency. Set when the robot is fully docked. | 
| `ALIENS!`=5 | Robot is or has been abducted. Will be very confused and disoriented after waking up. Set if the robot has been kidnapped. |

