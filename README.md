# Implementation of Dr. Turpin's Motion Planning algorithm for homogenous robots in Chapter 4 of Safe, Scalable, and Complete Motion of Large Team of Interchangeable Robots [https://core.ac.uk/reader/214180800]

## Implementation includes C-Capt and D-Capt
### C-Capt
C-Capt will launch centralized control of robots using Hungarian Algorithm for minimization of distance square among all participating robots
C-Capt algorithm may be run with: <br>
```
ros2 launch mt_impl C-Capt.py num_nodes:={number of goals} num_robots:={number of robots}
```
### D-Capt
D-Capt will launch de-centralized control of robots using randomized sort of robots to goals and will complete pairwise trades among participating robots in communication distance (distance square minimized)
D-Capt algorithm may be run with: <br>
```
ros2 launch mt_impl D-Capt0.py num_nodes:={number of goals} and num_robots:={number of robots}
```
### Modified D-Capt
I have created a modified version of Dr.Turpin's algorithm where the Hungarian algorithm is run on each each subset 'robot group'. There is a tradeoff between optimality in task completion and robot communication/computational capabilities. I see a future direction of this reserach include implementation with RL deciding where the necssity would be for making robot communications and in hungarian algorithm computations. This paper, written by Mr. Jiang and Dr. Lu, is provided as reference: https://proceedings.neurips.cc/paper_files/paper/2018/file/6a8018b3a00b69c008601b8becae392b-Paper.pdf
D-Capt modified algorithm may be run with: <br>
```
ros2 launch mt_impl D-Capt.py num_nodes:={number of goals} and num_robots:={number of robots}
```

The code is stil somewhat buggy for both versions of D-Capt in P2P robot communication through services. Updates will be made soon for fixes.


Built in Python Ros2 Iron
