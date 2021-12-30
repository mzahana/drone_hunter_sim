# drone_hunter_sim
This package is for simulating the autonomous drone hunter project in Gazebo+ROS environment
# Setup
* Make sure that you have NVIDIA graphics card and its drivers are installed
* Make sure that you have docker installed
* Copy the [docker_run_drone_hunter_sim.sh](https://github.com/riotu-lab/drone_hunter_sim/blob/main/scripts/docker_run_drone_hunter_sim.sh) content in a local script and make it executable using the command `chmod +x docker_run_drone_hunter_sim.sh`
* You will need to get the required `GIT_TOKEN` to be able to access RIOTU Github repos. Contact RIOTU GitHub admin for that
* Add the `GIT_TOKEN` to your PC `.bashrc` script, as follows
  ```sh
  export GIT_TOKEN=put_the_token_here
  ```
* Save & close `.bashrc` and source it `source ~/.bashrc`
* Run the `docker_run_drone_hunter_sim.sh` script
  ```sh
  ./docker_run_drone_hunter_sim.sh
  ```
* This should get you in the simulatoin docker container, after automatically installing all the neccessary software

# Run simulation
* To run drone hunting simulation of 2 drones,
  ```sh
  roslaunch drone_hunter_sim run_full_system.launch
  ```
  After the two drones takeoff, run the following to send target position estimates to the prediction system to complete the loop and start tracking!
  ```bash
  roslaunch drone_hunter_sim tf2odom_publisher.launch
  ```
* To run the basic simulation, but with one drone that does drone detection,
  ```sh
  roslaunch drone_hunter_sim drone_detection.launch
  ```
