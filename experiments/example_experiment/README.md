# Example Experiment
This is an example experiment configuration.

More information about the exeperiment setup can be found in the `Getting Started` -> `Experiment` section of the wiki.

In short, this folder contains all configuration files needed to run an experiment.

When executing an experiment using the crs_launch file, all used configuration files (*.yaml) all backed up into a new experiment folder (experiment_YYYY_MM_DD/HH_MM_SS/...). 

**DO NOT** synchronize all these experiment files over git.

In order to execute this experiment, launch:
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment
```

A few additional examples:

### Bypass State Estimation
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment bypass_estimator:=true
```
### Without RVIZ 
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment view_rviz:=false
```

### With Backtracker to recover from crashes
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment use_backtracker:=true
```

### With Custom Estimator config
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment simulator_config:=<absolute_path_to_config.yaml>
```

### With different Track
(Note, you will also need to adjust starting position in model.yaml. Track config is located at ros/tools/track_generation/tracks)
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment track_name:=LONG_TRACK
```