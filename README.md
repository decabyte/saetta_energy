# SAETTA: System for Adaptive Energy Tracking and Task Assessment 

This repo holds the ROS implementation of the SAETTA system. This work is still in progress.

## Requirements

The following packages are needed to run the software inside this repo:

  - jinja2
  - numpy 1.8+
  - scipy 0.14+
  - statsmodels 0.6+
  - sklearn 0.16+
  - control 0.7+

### Optional

  - gurobipy
  - pulp
  

### Thrusters Diagnostics

Thruster experiments can be run using the launch files available in this package and in the main `vehicle_core` one.
Here is a list of commands to set up the vehicle before running such experiments:
    
    rosparam set pilot/fault_control true                   # use false to disable this feature 
    roslaunch vehicle_core thruster_wrapper.launch
    roslaunch saetta_energy thruster_monitor.launch
    roslauch saetta_energy energy_monitor.launch

This will start the original pilot node in wrapped mode (allowing the injection of faults) and then the other nodes 
needed for the diagnostics experiments.
