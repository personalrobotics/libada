To run the python scripts for testing libada - 

*Additional Dependency:* [pr_assets](https://github.com/personalrobotics/pr_assets)

1. Launch the robot
    - Simulation
 	```
 	roslaunch libada simulation.launch
 	```
 	
    - Real robot (e.g. Jaco Gen2 6-DOF)
 	```
 	roslaunch libada default.launch version:=2 dof:=6
 	```

1. Run rviz
    ```
    rviz
    ```

1. Run the python script
 	```
 	source ~/<your workspace>/devel/setup.bash
 	python3 simple_trajectories.py
 	```

 	Change the ```IS_SIM``` flag to ```False``` when operating the real robot.
