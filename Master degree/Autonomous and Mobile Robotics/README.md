# Planning parking maneuvers for a car-trailer vehicle

This is the repository of the project proposed for the course of *Autonomous and Mobile Robotics*.

This work is centered on the paper by [Evestedt et al, **"Motion planning for a reversing general 2-trailer configuration using Closed-Loop RRT"** (IROS 2016)](https://www.researchgate.net/publication/306066065_Motion_planning_for_a_reversing_general_2-trailer_configuration_using_Closed-Loop_RRT), and it is based on the [**Open Motion Planning Library (OMPL)**](https://ompl.kavrakilab.org/).

## Compile and run the project

### Prerequisites

Please, [download the OMPL installation script](https://ompl.kavrakilab.org/install-ompl-ubuntu.sh) and follow the [instructions](https://ompl.kavrakilab.org/installation.html) from the OMPL documentation to install your desired version of the library:
```
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh [--python]/[--app]/[--github]
```
The script downloads and installs OMPL and all dependencies via *apt-get* & *pip* and from source.

### Compilation and execution

Continue downloading this repository as well. You can then compile and run the project with:
```
cd <your_path>/AMR_project/
make
./CarTrailerPlanning [--outfile <your_file.txt>, (default: solution.txt)]
                     [--graphfile <your_file.graphml>, (default: "")]
                     [--exp <forward, backward, obstacles, turn, real>, (default: forward)]
                     [--planner <RRT, SST, CL-RRT>, (default: RRT)]
                     [--run_count <int> (default: 1)]
                     [--run_limit <double> (default: 45.0)]
```
This will save the solution into the specified text file.

Optionally, you are free to save also the search tree computed during the execution of the program by passing to the command line a non-empty argument for ```--graphfile``` (ex. graph.graphml). This GraphML file can be parsed to plot the tree as explained in the next section.

### Create video from solution

We provide a procedure to convert the solution path to a video simulation. You are free to use the Python script as follows:
```
cd solution_to_video/
pip install -r requirements.txt
python solution_to_video [--input_path, (default: solution.txt)]
                         [--video_path, (default: solution_to_video.mp4)]
                         [--tree_path, (default: '')]
                         [--exp <forward, backward, obstacles, turn, real>, (default: '')]
```
This will create a video animation of the realized path.

Optionally, you can plot as a background image for the video a search tree saved before as follows:
```
cd solution_to_video/
python draw_tree [--graphml, (default: graph.graphml)]
                 [--outfile, (default: tree.png)]
                 [--exp <forward, backward, obstacles, turn, real>, (default: '')]
```
This will create an image of the search tree graph that can be used as the ```--tree_path``` argument of the ```solution_to_video.py``` script above.

## Experiments and results

The following experiments have been executed with the same parameter settings (please, refer to the [report](/report.pdf) for further details), each for a total amount of 50 runs collected through benchmarks and stored in the ```logs``` folder.

You can find some of these solution samples in the environment-related sections below.

### Experiment 1: Simple forward motion

**Description:** Forward motion in a free environment using two different planning algorithms.

|               |     <img src="https://render.githubusercontent.com/render/math?math=\x">     | <img src="https://render.githubusercontent.com/render/math?math=\y">  | <img src="https://render.githubusercontent.com/render/math?math=\theta"> | <img src="https://render.githubusercontent.com/render/math?math=\psi"> | <img src="https://render.githubusercontent.com/render/math?math=\phi"> |
| ------------- |:-------------:| -----:| -----:| -----:| -----:|
| Start     | -2 | -2 | 0 | 0 | 0 |
| Goal      | 2  |  2 | <img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}"> |  0 | 0 |

**Results:**


RRT             |  SST
:-------------------------:|:-------------------------:
<img src="/solution_to_video/solution_forward/rrt/solution_forward_rrt_path.gif" width="450" height="300"/>  | <img src="/solution_to_video/solution_forward/sst/solution_forward_sst_path.gif" width="450" height="300"/>

### Experiment 2: Simple backward motion

**Description:** Backward motion in a free environment using two different planning algorithms.

|               |     <img src="https://render.githubusercontent.com/render/math?math=\x">     | <img src="https://render.githubusercontent.com/render/math?math=\y">  | <img src="https://render.githubusercontent.com/render/math?math=\theta"> | <img src="https://render.githubusercontent.com/render/math?math=\psi"> | <img src="https://render.githubusercontent.com/render/math?math=\phi"> |
| ------------- |:-------------:| -----:| -----:| -----:| -----:|
| Start      | 2  |  2 | <img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}"> |  0 | 0 |
| Goal     | -2 | -2 | 0 | 0 | 0 |

**Results:**

RRT             |  SST
:-------------------------:|:-------------------------:
<img src="/solution_to_video/solution_backward/rrt/solution_backward_rrt_path.gif" width="450" height="300"/> | <img src="/solution_to_video/solution_backward/sst/solution_backward_sst_path.gif" width="450" height="300"/>

### Experiment 3: Circular obstacles avoidance

**Description:** Motion in the environment, populated with the circular obstacles, using SST planning algorithm with different optimization objectives. Planning in terms of minimizing the length of the path tends to steer very close to obstacles, which can sometimes be unsafe. For safety reasons, let's define an objective which attempts to steer the robot away from obstacles, path clearance objective, and compare the results.


|               |     <img src="https://render.githubusercontent.com/render/math?math=\x">     | <img src="https://render.githubusercontent.com/render/math?math=\y">  | <img src="https://render.githubusercontent.com/render/math?math=\theta"> | <img src="https://render.githubusercontent.com/render/math?math=\psi"> | <img src="https://render.githubusercontent.com/render/math?math=\phi"> |
| ------------- |:-------------:| -----:| -----:| -----:| -----:|
| Start      | -2  |  -2 | <img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{6}"> |  0 | 0 |
| Goal     | -2.1 | 1.5 | <img src="https://render.githubusercontent.com/render/math?math=\frac{7\pi}{8}">  | 0 | 0 |

**Results:**

SST (with min path length)             |  SST (with max clearance)
:-------------------------:|:-------------------------:
<img src="/solution_to_video/solution_obstacles/length/solution_obstacles_path.gif" width="450" height="300"/>  | <img src="/solution_to_video/solution_obstacles/clearance/solution_obstacles_path.gif" width="450" height="300"/>

### Experiment 4: Three point turn

**Description:** Corresponds to the *Three point turn* scenario from the reference paper. It is a motion in a narrow environment where an orientation inversion in a gap is required, using RRT, SST and CL-RRT planning algorithms.


|               |     <img src="https://render.githubusercontent.com/render/math?math=\x">     | <img src="https://render.githubusercontent.com/render/math?math=\y">  | <img src="https://render.githubusercontent.com/render/math?math=\theta"> | <img src="https://render.githubusercontent.com/render/math?math=\psi"> | <img src="https://render.githubusercontent.com/render/math?math=\phi"> |
| ------------- |:-------------:| -----:| -----:| -----:| -----:|
| Start      | 0.5  |  0.25 | <img src="https://render.githubusercontent.com/render/math?math=-\pi"> |  0 | 0 |
| Goal     | 2.5 | 0.25 | 0  | 0 | 0 |


**Results:**

RRT | SST        
:-------------------------:|:-------------------------:
<img src="/solution_to_video/solution_turn/rrt/solution_turn_rrt_path.gif" width="450" height="300"/> | <img src="/solution_to_video/solution_turn/sst/solution_turn_sst_path.gif" width="450" height="300"/> 

<h4 align="center">CL-RRT</h4>
<p align="center">
  <img width="450" height="300" src="/solution_to_video/solution_turn/clrrt/solution_turn_clrrt_path.gif">
</p>



### Experiment 5: Real parking test

**Description:** Corresponds to the *Real parking* scenario from the reference paper. It is a motion in a large environment with a very narrow parking gap, using RRT,SSt and CL-RRT planning algorithms.

|               |     <img src="https://render.githubusercontent.com/render/math?math=\x">     | <img src="https://render.githubusercontent.com/render/math?math=\y">  | <img src="https://render.githubusercontent.com/render/math?math=\theta"> | <img src="https://render.githubusercontent.com/render/math?math=\psi"> | <img src="https://render.githubusercontent.com/render/math?math=\phi"> |
| ------------- |:-------------:| -----:| -----:| -----:| -----:|
| Start      | 2  |  2.3 | <img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{4}"> |  0 | 0 |
| Goal     | 1.5 | 0.5 | <img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}">  | 0 | 0 |

**Results:**

RRT | SST        
:-------------------------:|:-------------------------:
<img src="/solution_to_video/solution_real/rrt/solution_real_rrt_path.gif" width="450" height="300"/> | <img src="/solution_to_video/solution_real/sst/solution_real_sst_path.gif" width="450" height="300"/> 

<h4 align="center">CL-RRT</h4>
<p align="center">
  <img width="450" height="300" src="/solution_to_video/solution_real/clrrt/solution_real_clrrt_path.gif">
</p>



## Authors
- Lorenzo Nicoletti - 1797464
- Leandro Maglianella - 1792507
- Olga Sorokoletova - 1937430
