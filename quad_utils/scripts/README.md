# Running tail scripts

## Simulation single
Run the shell file script in the following methods

```
./run_batch_simulation_single.py 11 <batch_num> <tail_type>
```

where batch_num is the batch it is ran (a way to keep up with the file) and tail type is whether to have no tail (0), nmpc tail (1), or simply feedback tail (2).


# TODO
Currently need to:
1) Find out how to execute a open-loop control for multi-link tail
2) Perhaps do some sort of feedback later
