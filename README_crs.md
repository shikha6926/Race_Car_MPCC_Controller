# CRS Project
[![pipeline status](https://gitlab.ethz.ch/ics-group/projects/andrea/crs-2.0/badges/dev/pipeline.svg)](https://gitlab.ethz.ch/carrona/crs/commits/dev)
[![License](https://img.shields.io/badge/License-BSD_2--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)
### Developers
[Sabrina Bodmer](@sabodmer) &nbsp; [David Helm](@helmd) &nbsp; [Rahel Rickenbach](@rrahel) &nbsp; [René Zurbruegg](@zrene) &nbsp;


### Principal Developers
[Jerome Sieber](@jsieber) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/mergedparrot.gif" width="25" height="25" /> &nbsp; [Simon Muntwiler](@simonmu) &nbsp; [Andrea Carron](@carrona) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/hd/laptop_parrot.gif" width="25" height="25" /> &nbsp;

### Getting Started
To get started with the CRS framework install it using the steps outlined in the [install wiki](https://gitlab.ethz.ch/ics-group/projects/andrea/crs-2.0/-/wikis/install). After the installation, have a look at the [first steps wiki](https://gitlab.ethz.ch/ics-group/projects/andrea/crs-2.0/-/wikis/setup/first-steps) and the rest of the [documentation](https://gitlab.ethz.ch/ics-group/projects/andrea/crs-2.0/-/wikis/home).

### Development
If you intend to develop code for this project, please read the [best practices](https://gitlab.ethz.ch/ics-group/projects/andrea/crs-2.0/-/wikis/development/best-practices) and [guidelines](https://gitlab.ethz.ch/ics-group/projects/andrea/crs-2.0/-/wikis/development/guidelines) first.

### TL;DR
1. [Docker] Build Docker Image and run it 
   ```bash 
   crs-docker up run
   ```
2. [Build] Build all packages 
   ```bash
   crs build
   ```
3. [Source] Source Workspace
   ```bash
   source devel/setup.bash
   ```
4. [Run] Run Experiment
   ```bash
   roslaunch crs_launch sim_single_car.launch experiment_name=example_experiment
   ```
#### Additional Examples

A few additional examples:

##### Bypass State Estimation
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment bypass_estimator:=true
```
##### Without RVIZ 
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment view_rviz:=false
```

##### With Backtracker to recover from crashes
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment use_backtracker:=true
```

##### With Custom Estimator config
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment simulator_config:=<absolute_path_to_config.yaml>
```

##### With different Track
(Note, you will also need to adjust starting position in model.yaml. Track config is located at ros/tools/track_generation/tracks)
```bash
 roslaunch crs_launch sim_single_car.launch experiment_name:=example_experiment track_name:=LONG_TRACK
```




### CRS Hall of Fame
[Christian Küttel](@kuettelc) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/hd/headsetparrot.gif" width="25" height="25" /> &nbsp; [Ben Tearle](@btearle) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/margaritaparrot.gif" width="30" height="22" /> &nbsp; [Robin Frauenfelder](@robinfr) &nbsp; <img src="https://cultofthepartyparrot.com/parrots/hd/docparrot.gif" width="25" height="25" /> &nbsp; [Daniel Mesham](@dmesham) &nbsp;
