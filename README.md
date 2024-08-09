# PID Planner Package
 
## Description
NOTE: This package was planned to be used for the 2024 Autonomous Navigation
Mission, but was not used during the mission due to a change of scope.

This package defines a ROS node used for controlling a rover's heading and
velocity using PID controllers. The node subscribes to the rover's position
and current goal and uses these values to calculate the correct command velocity
to send to the rover's drive motors.

## Known Limitations
This package was not used in the final version of the 2024 Autonomous Navigation
stack. There are multiple branches in this repository. The `main` branch is by
far the most thoroughly tested branch, and it preserves the main functionality
(travelling to a single waypoint without obstacle avoidance). This branch is
likely the most useful. The other branches in this repo each make attempts to
implement additional control functionality beyond simple point-to-point travel.
None of the branches besides `main` have been tested thoroughly, nor do they work
as intended.