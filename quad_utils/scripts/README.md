# Running tail scripts

## Simulation single
Run the shell file script in the following methods

```
./run_batch_simulation_single.py <env_type> <batch_num> <tail_type>
```

where:
- env_type is the environments with different elevation ranging from 0-11 (0 lowest, 11 highest)
- batch_num is the batch it is ran (a way to keep up with the file)
- tail type is whether to have no tail (0), nmpc tail (1), or simply feedback tail (2).


# TODO
Currently need to:
1) Find out how to execute a open-loop control for multi-link tail
2) Perhaps do some sort of feedback later
