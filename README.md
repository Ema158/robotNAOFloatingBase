## Matlab simulation of the NAO robot simulated as a floating base robot

This repository contains a Matlab code where the NAO robot is simulated as a floating base robot. Recursive algoritms based on Newton-Euler equations are used in 
order to simulate the dynamics. A Whole Body Controller obtains whole body trajectories by an optimization problem which track a desired task at the level of the center 
of mass.

Overview

* Dynamics of a multy rigid body robot with floating base are obtain by recursive algorithms.
* The complete dynamics of the robot projected at the center of mass are obtained by the centroidal model.
* A quadratic programming problem obtain whole body trajectories for a desired task.
* Friction and balance constraints are considered in the optimization problem.
