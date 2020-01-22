# Particle Filter Implementation for Vehicle Localization
<img src="media/ParticleFilterGIF.gif" width="700" height="500" />

## Introduction
In this project, a particle filter is implemented to localize a vehicle in a map. The vehicle is initialized in the map at an arbitrary location. This type of situation is commonly referred to as the kidnapped robot problem. The simulation environment is shown in the GIF above. The blue car represents the exact position of the vehicle in the simulator and the blue circle around it represents the vehicle location estimated by the particle filter. The 'X's in the map represent the landmark location and the green lines are the LiDAR measurements to the landmarks. The vehicle travels in a predetermined trajectory in the map. The simulator also shows the error in localization by comparing the actual location of the vehicle to the location calculated by the particle filter.

The initial estimation of the vehicle location is measured using GPS by taking into consideration the measurement error. The particles for the particle filter are initialized within the area enclosed by the GPS measurement and its covariance. The prediction step of the particle filter is implemented using the vehicle motion model. A constant turn-rate and velocity (CTRV) motion model is used in this implementation. The measurement step of the particle filter is implemented by assigning a measurement to each particle and performing proper data association. The measurements are with respect to the vehicle's frame of reference and the particles are initialized in the map's reference frame. A coordinate transformation step is applied to assign the measurement to each particle taking into account the measurement uncertainty. A Nearest-Neighbour (NN) approach is used to associate the measurement data to the landmarks in the map. Depending on the measurement a weight is assigned to the particles proportionally. The last step in the process is the resampling step. In this step, the particles are sampled proportionally to their weights with replacement. The resampling wheel approach is used in this step which allows the particles with maximum weight to be sampled multiple times and the entire process repeats itself.
## Project Build Instructions
### Ubuntu
```bash
git clone
cd Kidnapped-Vehicle
mkdir build && cd build
cmake ..
make
./particle_filter
```
## Build Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * run `./install-ubuntu.sh` or `./install-mac.sh` depending on your platform
* Simulator
  * The simulator can be downloaded from the [Udacity GitHub Repository](https://github.com/udacity/self-driving-car-sim/releases)

## References
1. The starter code and the simulator is adopted from the [Udacity GitHub Repository](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)
