<h1 align="center">SMART: Self-Morphing Adaptive Replanning Tree</h1>
<h4 align="center">Zongyuan Shen, James P. Wilson, Shalabh Gupta*, Ryan Harvey</h4>
<h4 align="center">Department of Electrical & Computer Engineering, University of Connecticut, Storrs, CT, USA</h4>

<p align="center"> [<b><a href="https://arxiv.org/abs/2305.06487">Paper</a></b>] &emsp; [<b><a href="https://docs.google.com/viewer?url=https://raw.githubusercontent.com/degoes-consulting/lambdaconf-2015/master/speakers/jdegoes/intro-purescript/presentation.pdf">Presentation</a></b>] &emsp; [<b><a href="#citation">Citation</a></b>] </p>

## Table of Contents

- [Introduction](#Introduction)
- [Demo](#Demo)
- [Usage](#usage)
- [Citation](#Citation)
- [Acknowledgement](#Acknowledgement)
- [License](#license)

## Introduction 
The paper presents an algorithm, called Self-Morphing Adaptive Replanning Tree (SMART), that facilitates fast reactive replanning in dynamic environments. SMART performs risk-based tree-pruning if the current path is obstructed by nearby moving obstacle(s), resulting in multiple disjoint subtrees. Then, for speedy recovery, it exploits these subtrees and performs informed tree-repair at hot-spots that lie at the intersection of subtrees to find a new path. Please refer to [our paper](https://arxiv.org/abs/2305.06487) for more detailts.

![Main idea](https://github.com/ZongyuanShen/SMART/assets/136994172/e68db789-7cf1-4b97-bb22-eb7e0b036c44)

<p align="center">
Fig. Illustration of the SMART algorithm: a) tree-pruning and disjoint tree creation, and b)-i) tree-repair and replanning.
</p>

## Demo 
### Scenario 1 with Dynamic Obstacles
This scenario consists of a 32m by 32m space populated with 15 dynamic obstacles moving at a constant speed of 4m/s.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=nTQUwghvy5Q" target="_blank">
 <img src="https://github.com/ZongyuanShen/SMART/assets/136994172/ff1d5344-f0ca-47e4-a9aa-61d383ac4f56" alt="Watch the video"  width="240" height="180" border="10" />
</a>

### Scenario 2 with Static and Dynamic Obstacles
This scenario depicts a real situation (e.g., a factory) with both static and dynamic obstacles. It consisted of a 66m by 38m space with a static obstacle layout and 10 dynamic obstacles. Each obstacle moves at a different speed selected from the set {1, 2, 3, 4}m/s.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=nTQUwghvy5Q" target="_blank">
 <img src="http://img.youtube.com/vi/nTQUwghvy5Q/mqdefault.jpg" alt="Watch the video" width="240" height="180" border="10" />
</a>

### Experiment
The SMART algorithm is further validated by real experiments in a 7m by 7m lab space with both static and dynamic obstacles. A robot called ROSMASTER X3 is used that is equipped with 1) a RPLIDAR S2L lidar with a range of 8m for obstacle detection, 2) MD520 motor with encoder for detection of rotation angle and linear displacement, and 3) MPU9250 IMU for detection of speed, acceleration, and orientation. An Extended Kalman Filter is used to fuse data from the IMU and motor encoder for localization.


https://github.com/ZongyuanShen/SMART/assets/136994172/71cbc6b4-fd0b-432b-84b3-03f6d5acf87c


## Usage

```sh
compile Main.cpp
run the executable file
```

## Citation

If you use the results presented in this paper or the code from the repository, please cite the relevant [paper](https://arxiv.org/abs/2305.06487):
```
@article{shen2023smart,
title={SMART: Self-Morphing Adaptive Replanning Tree},
author={Shen, Zongyuan and Wilson, James P and Gupta, Shalabh and Harvey, Ryan},
journal={arXiv preprint arXiv:2305.06487},
year={2023}
}
```

## Acknowledgement
This research is supported by the Air Force Research Laboratory.

## License

[MIT](LICENSE) © Zongyuan Shen
