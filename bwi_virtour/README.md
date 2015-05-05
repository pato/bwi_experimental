BWI Virtour
===========

Virtour is a public facing system for teleoperated building tours.
On the robot: uses ROS to manage tour, stream video, and take commands.
On the website: uses rosjs to pipe commands to the robot

## Features

* Adaptive Video streaming
* Teleoperated servo control
* Goal based logical navigation
* In-place robot rotation
* Robot position marker
* Tour leader management

## Instructions

On robot:

1. Set up smallDNS

2. Bring up robot (eg: `roslaunch bwi_launch segbot_v2.launch`)

3. Launch `roslaunch bwi_virtour virtour.launch`

On server:

1. Place repo in `/var/www`

2. Modify `index.html` to add robot

## ROS Nodes

* `tour_manager` - tour leader state machine
* `go_to_location_service_node` - logical navigation commands
* `rotation_service_node` - rotation commands

### Tour Manager

Manages the tour state machine. Handles tour leaders, when
tours are allowed, and authentication.

### Logical Navigation

Handled by the `go_to_location_service_node`.
To add new goals, edit `js/virtour.js`

### Rotation

Handled by `rotation_service_node`. Takes in a float for the rotation delta

## Project structure

* `/web` contains all files necessary for hosting the website
* `/src` contains the c++ source for the ROS nodes
* `/srv` contains the service definitions


## Resources

[Old Talk](https://docs.google.com/presentation/d/1cNeUuevuT522KYIJN_F945JEqcbchYooTK5Sci0EaNs/edit?usp=sharing)
[Small DNS](https://github.com/pato/smallDNS)
