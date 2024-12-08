<h1 align="center">SMART: Self-Morphing Adaptive Replanning Tree</h1>

<p align="center"> [<b><a href="https://ieeexplore.ieee.org/document/10250928">Paper</a></b>] &emsp;  [<b><a href="https://zongyuanshen.github.io/projects/2_project/">Project Page</a></b>] &emsp; [<b><a href="https://www.youtube.com/playlist?list=PL4xQ0coJXyn97zfJDkQchZNPpYGJfCBJ6">Video</a></b>] &emsp;[<b><a href="https://drive.google.com/file/d/1d_cqbyHNAHxAA4SC-DgQBfWWJfAHBIod/view?usp=drive_link">Slide</a></b>] &emsp;[<b><a href="#citation">Citation</a></b>]

## Table of Contents

- [Introduction](#Introduction)
- [Usage](#usage)
- [Citation](#Citation)
- [Acknowledgement](#Acknowledgement)
- [License](#license)
- [Maintaince](#Maintaince)

## Introduction 
**SMART** facilitates fast reactive replanning in dynamic environments. It performs risk-based local tree-pruning if the current path is obstructed by nearby moving obstacle(s), resulting in multiple disjoint subtrees. Then, it exploits these subtrees and performs informed tree-repair at hot-spots that lie at the intersection of subtrees to find a new path.

<p align="center">
  <img src="Gif/Scenario1_video_1x.gif" height = "255"/>
  <img src="Gif/Scenario2_video_1x.gif" height = "255"/>
</p>

<p align="center">
Fig. Point robot (yellow point) and dynamic obstacles (grey circle) move at a constant speed of 4m/s.
</p>

## Usage

### C++
- **User-defined inputs:**
  - File "Main.cpp": trialIndex, dynObsNum, dynObsSpeed, sceneIndex, dynObsPosition, robotInitState, goalState
  - File "SMART.cpp": cellSize, staticObsMap.txt
```
trialIndex: seed for random sample generation
sceneIndex: seed for dynamic obstacle trajectory generation
dynObsPosition: initial position of dynamic obstacle
goalState: goal position
robotInitState: contain initial robot position and constant linear speed
cellSize: size of cell in meter
staticObsMap.txt: binary occupancy grid map. free = 0; occupied = 1.
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
  - "folder": It is a character array to store the directory to the folder that contains the recorded data.
  - "videoRecord": A demo video will be created if videoRecord = true.

- **Demo generation:**
```
run Main.m
```
    

## Citation

If you use the results presented in this paper or the code from the repository, please cite this paper:
```
@article{shen2023smart,
title={SMART: Self-Morphing Adaptive Replanning Tree},
author={Shen, Zongyuan and Wilson, James P and Gupta, Shalabh and Harvey, Ryan},
journal={IEEE Robotics and Automation Letters},
year={Sep. 2023},
volume={8},
number={11},
pages={7312-7319}
}
```
Please kindly star :star: this project if it helps you.

## Acknowledgement
This research is supported by the Air Force Research Laboratory.

## License

[MIT](LICENSE) © Zongyuan Shen
