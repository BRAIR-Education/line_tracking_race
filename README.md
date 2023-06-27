# Line Tracking Race

Repo with the base ros package for the "line tracking race" project.

## How to launch the Simulation

Type in your terminal:
```
roslanch line_tracking_race race_track.launch
```

## How to control the Vehicle

It is possible to control the 2 "motors" of the front wheels of the vehicle, publishing 2 topics named `/car/front_left_velocity_controller/command` and `/car/front_right_velocity_controller/command`, that contain `std_msgs/Float64` as ROS msg type.

The controller is a PID on the velocity of the wheel. The gain for this controller are defined in the file *./config/car_control.yaml*
