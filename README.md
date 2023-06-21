# Self-triggered Controller

## Publication

G. Yang, C. Belta and R. Tron, "Self-triggered Control for Safety Critical Systems Using Control Barrier Functions," 2019 American Control Conference (ACC), Philadelphia, PA, USA, 2019, pp. 4454-4459, doi: 10.23919/ACC.2019.8814657.

[ACC Paper](https://ieeexplore.ieee.org/document/8814657)

[Arxiv Paper](https://arxiv.org/abs/1903.03692)


## Software Requirement
* Ubuntu 18.04
* Matlab 2018a

* Mac OS 13
* Matlab 2018a



## Files Explanation
*simulation.m* contains main simulation codes for running both self-triggered control example or periodic control example

*QPcontroller.m* formulates CLF-CBF QP for at a given update instance, the problem is solved by Matlab's

*clf_plot.m* plots the value of CLF constraint for each given state and control input

*stateTraj_plot.m* plots state and control trajectories with respect to time

*rBound.m* calculates r(t) that bounds system's trajecotry given an initial time t_k=0

*sys_dynamics.m* contains the dynamics of double-integrator system
## Example

Open *simulation.m* and run the first block to define *problem* structure. Next, run either second block (self-triggered control) or third block (periodic control). The simulation results are saved in **matfile_storage** directory as mat files.

To get the state trajectory plots, run *stateTraj_plot(problem,conMode)* in Command Window with flag *conMode=1* for self-triggered control or *conMode=2* for periodic control.

To get CLF constraint value plots for a given holding period (either fixed period or dynamic period), run *clf_plot*, with the same flag *conMode* setting as shown above.

