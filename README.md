# Group 7 Robotics Coursework

## Instructions for Development
Do not make changes directly on the main branch. Before making changes please do the following:
- First checkout the main branch using ```git checkout main```
- Run ```git pull``` to update your local repository with the latest changes
- To start making your changes, create a new branch and checkout (navigate) to it using ```git checkout -b <BRANCH_NAME>```
- After commiting your changes to your local branch, to push your changes to the remote run ```git push --set-upstream origin <BRANCH_NAME>```
- Go to GitLab and create a Pull Request to merge your local <BRANCH_NAME> into main
- At this point ask another group member to approve your PR to ensure your changes are valid
- If a merge conflict arises ask Evan :)
- If all is fine, complete the PR and your changes will be merged onto main

## Running the Robot
In order to run the robot and identify the correct characters, carry out the following:
- Activate the ROS image using ```singularity shell --nv /vol/scratch/SoC/COMP3631/ros.simg```
- Navigate into the catkin workspace and run ```source devel/setup.bash```
- Set the active world file using ```export TURTLEBOT_GAZEBO_WORLD_FILE=<ABSOLUTE_PATH_TO_REPO>/world/project.world```
- Start Gazebo using ```[vglrun] roslaunch turtlebot_gazebo turtlebot_world.launch```
- In another terminal, navigate to ```<ABSOLUTE_PATH_TO_REPO>/launch``` and run ```roslaunch simulated_localisation.launch map_file:=<ABSOLUTE_PATH_TO_REPO>/world/map/project_map.yaml``` to setup the localisation module and load the correct map file
- In another terminal, navigate to ```<ABSOLUTE_PATH_TO_REPO>``` and run ```[vglrun] roslaunch turtlebot_rviz_launchers view_navigation.launch``` to launch RVIZ
- Navigate to ```<ABSOLUTE_PATH_TO_REPO>``` and run ```chmod +x src/main.py```
- Finally, run your code as usual using ```rosrun group_project main.py```
