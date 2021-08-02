# HopperRRT
Data files for paper Feedback motion planning of legged robots by Zamani et al. ICRA 2018

Refer to the paper (also included in this folder) for reference. 

YouTube video of the final result: https://youtu.be/Ie-WGqAl6-4

Paper
@inproceedings{zamani2019feedback,
  title={Feedback motion planning of legged robots by composing orbital lyapunov functions using rapidly-exploring random trees},
  author={Zamani, Ali and Galloway, Joseph D and Bhounsule, Pranav A},
  booktitle={2019 International Conference on Robotics and Automation (ICRA)},
  pages={1410--1416},
  year={2019},
  organization={IEEE}
}

Part A)
Reference V Method A Trajectory optimization. These files are not provided. Here a trajectory optimization is solved to get controls u for a given state x that achieves a given decrease in Lyapunov function. e.g., V(k+1) = 0.1 V(k).

These files produced data_lyap1_09_xy_pq.mat where the xy indicates the speed x.y and pq indicates the height as p.q
For example data_lyap1_09_20_13.mat means that this file found u for state x around the fixed point 2.0 (speed) and 1.3 (height).


Part B)
Run main.m with options = 32 (defined in the first few lines). 
	This uses the mapping control u = F(x)  (x=state) for multiple ROAs and fits a single neural network. See Fig. 3 in the paper.
	Input files: data_lyap1_09_xx_xx.mat (multiple files)
	Output: control_policy.mat


Part C)
Run RRT_ellipse_dynamic.m 
	This uses the control_policy.mat which has u = F(x,ellipse parameters) and RRT to do motion planning. See Algorithm 1 and Figure 4 in the paper.
	Input files: control_policy.mat
	Output: A movie showing transitions
