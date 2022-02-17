# Minimum-Time Quadrotor Waypoint Flight in Cluttered Environments

This repository contains the code for the Minimum-Time Quadrotor Waypoint Flight in Cluttered Environments presented in our paper: [Penicka and Scaramuzza RA-L'22](https://rpg.ifi.uzh.ch/docs/RAL22_Penicka.pdf).

[![illustration](./docs/RAL22_Penicka.jpg)](https://youtu.be/TIvvHtzRwSo)



## Citing
If you use this code in an academic context, please cite the following publication:

R. Penicka, D. Scaramuzza, **Minimum-Time Quadrotor Waypoint Flight in Cluttered Environments**, IEEE Robotics and Automation Letters, 2022. ([PDF](https://rpg.ifi.uzh.ch/docs/RAL22_Penicka.pdf))

```
@article{penicka22RALmintimeplanning,
  author={Penicka, Robert and Scaramuzza, Davide},
  journal={IEEE Robotics and Automation Letters}, 
  title={Minimum-Time Quadrotor Waypoint Flight in Cluttered Environments}, 
  year={2022}
}
```

## License 
GPL-3.0 License. Copyright (C) 2022 R. Penicka, D. Scaramuzza (Robotics and Perception Group, University of Zurich).

This is a research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation and usage

The code has been developed and tested on Ubuntu 20.04.

### Downloading the code

Clone the code repository and update the submodules.<br />
`git clone https://github.com/uzh-rpg/sb_min_time_quadrotor_planning.git`<br />
`cd sb_min_time_quadrotor_planning`<br />
`git submodule update --init --recursive`

### Compilation and dependencies

Install following dependencies:<br />
`sudo apt-get install build-essential cmake pkg-config ccache zlib1g-dev libomp-dev libyaml-cpp-dev libhdf5-dev libgtest-dev liblz4-dev liblog4cxx-dev libeigen3-dev python3 python3-venv python3-dev python3-wheel python3-opengl`<br />

Compile first the dependencies present in submodules using<br />
`make dependencies`<br />

Afterwards you can compile the code using<br />
`make`

### Maps preparation

To create the ESDF maps from the mesh .obj fles use the map.py script in the python folder. To have all the dependencies we suggest using the python environment.
Start the environment and activate it using:<br />
`python3 -m venv env`<br />
`source env/bin/activate`<br />
Afterwards install the python dependencies using pip (or pip3 if python3 is not default):<br />
`pip install wheel`<br />
`pip install pyopengl==3.1.0`<br />
`pip install numpy trimesh matplotlib mesh_to_sdf python-csv`

When the dependencies are installed run the following commands in the python folder to create the ESDF maps.
`./map.py ../blender/random_columns.obj`<br />
`./map.py ../blender/repairing_office.obj`<br />
`./map.py ../blender/arena_track_obstacles_multistory.obj`

### Running the code

After compilation you should see the main binary. The main cofiguration file is the sst.yaml where the desired waypoints and map can be set. The drone.yaml file includes configuration of the used quadrotor. The found trajectory is afterwords in file path_dense.csv. To run the code simply run command:<br />
`./main`
