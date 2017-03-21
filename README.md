# Extended Kalman Filter
This project fuses the position and velocity measurements of obstacles from rader and lasar measurements to track the obstacles through time. It manages both cartesian and polor coordinates. 
---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Results

The Kalman Filter was able to track obstacles fairly accuractely with the sample measurements/ground truth that I used. 

The position predictions were most accurate across different datasets. 
![Tracking of Position Over Time (A)](resources/pos1_graph.png?raw=true "Data Analysis")

![Tracking of Position Over Time (B)](resources/pos_graph.png?raw=true "Data Analysis")

There was some noise in the velocity predictions, mostly caused by the radar measurements.
![Tracking of Velocity Over Time (A)](resources/vx1_graph.png?raw=true "Data Analysis")

![Tracking of Velocity Over Time (B)](resources/vx_graph.png?raw=true "Data Analysis")
