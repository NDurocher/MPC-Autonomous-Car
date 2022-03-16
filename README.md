# MPC Autonomous Car

## Motivation
This project was completed as part of the Classical Autonmous Systems class during the fall of 2020 at the University of Southern Denmark. The final presentation is available [here](FinalPresentation.pdf)

## Screenshots
<img src="./Overleaf pictures/MPC Plot.png" width="854" height="480" />

## Features
This project implements a time-horizon MPC controller for a theoretical autonomous car in MATLAB. The controller utilizes accelertion and steering angle to predict a trajectory 12 timesteps ahead. The inputs are then optimized to reduce the energy usage, improve controll smoothness and minimize both cross path error and waypoint heading error. 

## How to use?
Run MPCtest.m and a figure will being plotting. The upper figure is a global map where the car's path is represented by a blue line. The Red line is the center of the track while the Black lines are the edges. The lower figure is the predicted optimal tracjectory in local coordinates.
