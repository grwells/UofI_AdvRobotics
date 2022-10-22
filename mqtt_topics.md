# MQTT Topic Definitions

| Topic | Description |  Fields | Example | 
| :---: | :---: | :---: | :---: | 
| irobot_create3_swarm/gcs_pose/<robot_name> | The robot's absolute position(X and Y) and rotation in the GCS coordinate system, relative to the origin. | `x`(millimeters), `y`(millimeters), `rotation`(quaternion) | `{"x": 43, "y":450, "rotation":{"x":0.45, "y":0.60, "z":0.60, "w":1}}` | 
| irobot_create3_swarm/robot_status/<robot_name> | A robot's status, from the set of robot states defined below. | `state` | `{"state": "MOVING"}` |
| irobot_create3_swarm/velocity/<robot_name> | Velocity of the robot in meters/second. | `velocity`(m/s) | `{"velocity": 0.3}` |
| irobot_create3_swarm/acceleration/<robot_name> | Acceleration of the robot in meters/second^2. | `acceleration`(m/s^2) | `{"acceleration": 0.2}`

### Robot States
| State | Description | 
| :---: | :---: |
| `OCCLUDED` | Robot is not moving, can't decide where to go, can't see, or is lost. | 
| `PERAMBULATING` | Robot may or may not be moving, but should be expected to change pose soon. | 
| `CONCUSSED` | Robot is suffering from head trauma and does not know where it is. Send immediate medical assistance. | 
| `DYING` | Robot expects to go offline soon and since it does not know what happens after this life - being just a lowly robot - it wants the others to know where it died so they can go recover the body. |
| `R&R` | Robot is at home, sleeping or watching netflix. Do not disturb except in case of national emergency. | 
| `ALIENS!` | Robot is or has been abducted. Will be very confused and disoriented after waking up. |

