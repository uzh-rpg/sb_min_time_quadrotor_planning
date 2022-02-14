## Minimum-Time Quadrotor Waypoint Flight in Cluttered Environments

Code for paper: 
R. Penicka, D. Scaramuzza, "Minimum-Time Quadrotor Waypoint Flight in Cluttered Environments", in Robotics and Automation Letters (RAL), 2022

### Downloading the code
Clone the code repository and update the submodules.<br />
`git clone https://github.com/uzh-rpg/sb_min_time_quadrotor_planning.git`<br />
`cd sb_min_time_quadrotor_planning`<br />
`git submodule update --init --recursive`

### Compilation and dependencies

Install following dependencies:<br />
`sudo apt-get install libyaml-cpp-dev `<br />

Compile first the dependencies present in submodules using<br />
`make dependencies`<br />

Afterwards you can compile the code using<br />
`make`

### Maps preparation

`./map.py ../blender/random_columns.obj`<br />
`./map.py ../blender/repairing_office.obj`<br />
`./map.py ../blender/arena_track_obstacles_multistory.obj`

### Running the code

After compilation you should see the main binary. The main cofiguration file is the sst.yaml where the desired waypoints and map can be set. The drone.yaml file includes configuration of the used quadrotor. The found trajectory is afterwords in file path_dense.csv.