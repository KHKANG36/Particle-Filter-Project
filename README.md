## Particle filter project for vehicle localization 
This is a project for Udacity Self-Driving Car Nanodegree program. In this project, I implemented a particle filter to localize the kidnapped(Do not know where I am) vehicle. All codes are written with C++ and tested on the Udacity simulator. 

## Requirement 
- C++
- Udacity simulator : [here](https://github.com/udacity/self-driving-car-sim/releases)
- uWebSocketIO : [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac
- For window users : [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Run the Project 
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

## About the Project 
I constructed the C++ algorithm mostly based on the lecture, but several lines of codes & algorithm was implemented from the scratch.
Based on the criteria of Udacity simulator, I passed the particle filter with the performance of : 
(Error on x direction : .107, y direction : .103, yaw angle : .003). 
This is my result image:  
![Test image](https://github.com/KHKANG36/Particle-Filter-Project/blob/master/particle_filter.png)


## Discussion/Issues 
The accuracy was increased with the increased number of particles, however, only to the certain limit. 
In my code, I cannot help making a lot of "for" loops because the algorithm requires many comparisons and coordination matching. 
Next step would be to find the more efficient algorithm for this particle filter implementation. 
