# Running tail scripts

## Simulation single
Run the shell file script in the following methods

```
roscd quad_utils/scripts
chmod +x run_batch_simulation_single.py
./run_batch_simulation_single.py <env_type> <batch_num> <tail_type>
```

where:
- env_type is the environments with different elevation ranging from 0-11 (0 lowest, 11 highest)
- batch_num is the batch it is ran (a way to keep up with the file)
- tail type is whether to have no tail (0), nmpc tail (1), or simply feedback tail (2).

To run without script:
```
roslaunch quad_utils quad_gazebo.launch tail:=true tail_type:=1
roslaunch quad_utils standing.launch
```

## Run Double Pendulum Quadruped Tail Control Scheme
```
roscd quad_utils/scripts
./run_batch_simulation_single.py <env_type> <batch_num> 3
```


# TODO
Currently need to:
1) Find out how to execute a open-loop control for multi-link tail
2) Form dynamic model
3) Get data & results
