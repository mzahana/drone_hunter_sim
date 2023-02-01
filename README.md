# drone_hunter_sim
This package is for simulating the autonomous drone hunter project in Gazebo+ROS environment
# Setup
* Make sure that you have NVIDIA graphics card and its drivers are installed
* Make sure that you have docker installed
* Copy the [docker_hunter_sim_noetic.sh](https://github.com/mzahana/drone_hunter_sim/blob/noetic/scripts/docker_hunter_sim_noetic.sh) content in a local script and make it executable using the command `chmod +x docker_hunter_sim_noetic.sh`
* You will need to get the required `GIT_TOKEN` to be able to access the Github repos. Contact the GitHub repo admin for that
* Add the `GIT_TOKEN` to your PC `.bashrc` script, as follows
  ```sh
  export GIT_TOKEN=put_the_token_here
  ```
  You can also add it to your `$HOME/.bashrc` so you don't have to `export` is every time you opent a new terminal window.
  ```bash
  echo "export GIT_TOKEN=put_the_token_here" >> $HOME/.bashrc && source $HOME/.bashrc
  ```
* You need to add your Github user ID as an environment variable. Add the `GIT_USER` to your PC's `.bashrc` script, as follows
  ```sh
  export GIT_USER=your_user_id
  ```
  You can also add it to your `$HOME/.bashrc` so you don't have to `export` is every time you opent a new terminal window.
  ```bash
  echo "export GIT_USER=your_user_id" >> $HOME/.bashrc && source $HOME/.bashrc
  ```

* Run the `docker_run_drone_hunter_sim.sh` script
  ```sh
  ./docker_run_drone_hunter_sim.sh
  ```
* This should get you in the simulation docker container, after automatically installing all the neccessary software in the shared folder located in your `$HOME` directory. The shared folder name `hunter_sim_noetic_shared_volume`, and all the ROS pakckages are located there. This is shared between the docker container and your PC file system

**NOTE** If you would like to run VS Code inside the container, there is an alias that is already created for you during the setup. You can just exeute the following command inside the container `code_ws`

**NOTE** To enter a new terminal inside the simulation container, just execute the `docker_run_drone_hunter_sim.sh` again.

# Run simulation
* To run drone hunting simulation of 2 drones, with drone detection (set `show_detection_imgs:=True`)
  ```sh
  roslaunch drone_hunter_sim run_full_system.launch show_detection_imgs:=True
  ```
**NOTE** When you run the simulation for the first time, Gazebo might take some time to donwload some models from the internet. Just be patient !

**NOTE** currently the target prediction is not working properly, so the MPC (controller) is deactivated in the `run_full_system.launch` file.
