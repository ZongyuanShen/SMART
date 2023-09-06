<h1 align="center">SMART: Self-Morphing Adaptive Replanning Tree</h1>
<h4 align="center">Zongyuan Shen, James P. Wilson, Shalabh Gupta*, Ryan Harvey</h4>
<h4 align="center">Department of Electrical & Computer Engineering, University of Connecticut, Storrs, CT, USA</h4>

<p align="center"> [<b><a href="https://arxiv.org/abs/2305.06487">Paper</a></b>] &emsp; [<b><a href="https://docs.google.com/viewer?url=https://raw.githubusercontent.com/degoes-consulting/lambdaconf-2015/master/speakers/jdegoes/intro-purescript/presentation.pdf">Presentation</a></b>] &emsp; [<b><a href="#Demo">Demo</a></b>] &emsp; [<b><a href="#citation">Citation</a></b>]</p>

## Table of Contents

- [Introduction](#Introduction)
- [Demo](#Demo)
- [Usage](#usage)
- [Citation](#Citation)
- [Acknowledgement](#Acknowledgement)
- [License](#license)
- [Maintaince](#Maintaince)

## Introduction 
**SMART** facilitates fast reactive replanning in dynamic environments. It performs risk-based tree-pruning if the current path is obstructed by nearby moving obstacle(s), resulting in multiple disjoint subtrees. Then, for speedy recovery, it exploits these subtrees and performs informed tree-repair at hot-spots that lie at the intersection of subtrees to find a new path.

<p align="center">
  <img src="Gif/Scenario1_video_1x.gif" height = "300"/>
  <img src="Gif/Scenario2_video_1x.gif" height = "300"/>
</p>

<p align="center">
Fig. Robot and dynamic obstacles are moving at a constant speed of 4m/s. Video play speed is 1x.
</p>

## Demo 
### Scenario 1 with Dynamic Obstacles
This scenario consists of a 32m by 32m space populated with 15 dynamic obstacles moving at a constant speed of 4m/s. The robot is moving at a constant speed of 4m/s.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Xb0yWwwN0SE
" target="_blank"><img src="http://img.youtube.com/vi/Xb0yWwwN0SE/mqdefault.jpg" 
alt="IMAGE ALT TEXT HERE"  border="10" /></a>

### Scenario 2 with Static and Dynamic Obstacles
This scenario depicts a real situation (e.g., a factory) with both static and dynamic obstacles. It consisted of a 66m by 38m space with a static obstacle layout and 10 dynamic obstacles. Each obstacle moves at a different speed selected from the set {1, 2, 3, 4}m/s. The robot is moving at a constant speed of 4m/s.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Xb0yWwwN0SE
" target="_blank"><img src="http://img.youtube.com/vi/Xb0yWwwN0SE/mqdefault.jpg" 
alt="IMAGE ALT TEXT HERE"  border="10" /></a>

### Experiment
The SMART algorithm is further validated by real experiments in a 7m by 7m lab space with both static and dynamic obstacles. A robot called ROSMASTER X3 is used that is equipped with 1) a RPLIDAR S2L lidar with a range of 8m for obstacle detection, 2) MD520 motor with encoder for detection of rotation angle and linear displacement, and 3) MPU9250 IMU for detection of speed, acceleration, and orientation. An Extended Kalman Filter is used to fuse data from the IMU and motor encoder for localization.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Xb0yWwwN0SE
" target="_blank"><img src="http://img.youtube.com/vi/Xb0yWwwN0SE/mqdefault.jpg" 
alt="IMAGE ALT TEXT HERE"  border="10" /></a>

## Usage

### C++
- **User-defined inputs:**
  - File "Main.cpp": trialIndex, dynObsNum, dynObsSpeed, robotSpeed, sceneIndex, dynObsPosition 
  - File "SMART.cpp": goalX, goalY, cellSize, robotX, robotY
  - File "SMART.h": mapROW, mapCOL
```
trialIndex: control the random seed to generate the tree
sceneIndex: control the random seed to generate dynamic obstacle motion
dynObsPosition: initial position of dynamic obstacle
goalX and goalY: fixed goal position
robotX and robotY: varying robot position with user-defined initial value
cellSize: size of cell in meter
mapROW: number of cells in each row
mapCOL: number of cells in each column
```

- **Outputs:**
  - Replanning time (s)
  - Trajectory length (m)
  - Travel Time (s)
  - Recorded data
    
- **Recorded data:**
  - Dynamic obstacle
  - Tree
  - Path
  - Robot's footprint
```
Function dataRecord() is used to record the data for visualization on Matlab.
```

- **Compilation:**
```
compile Main.cpp
run the executable file
```

### Matlab for visualization:
- **User-defined input:**
  - File "Main.m": folder. It is a character array to store the directory to the folder that contains the recorded data.
  - File "Main.m": videoRecord. A demo video will be created if videoRecord = true.

- **Demo generation:**
```
run Main.m
```
    

## Citation

If you use the results presented in this paper or the code from the repository, please cite the relevant [paper](https://arxiv.org/abs/2305.06487):
```
@article{shen2023smart,
author={Shen, Zongyuan and Wilson, James P and Gupta, Shalabh and Harvey, Ryan},
journal={IEEE Robotics and Automation Letters},
title={SMART: Self-Morphing Adaptive Replanning Tree},
year={2023}
volume={},
number={},
pages={-},
doi={}
}
```

## Acknowledgement
This research is supported by the Air Force Research Laboratory.

## License

[MIT](LICENSE) © Zongyuan Shen

## Maintaince
For any technical issues, please contact Zongyuan Shen (zongyuan.shen@uconn.edu).
