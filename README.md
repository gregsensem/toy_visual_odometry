# toy_visual_odometry
This is a toy visual odometry built for practice purpose. It referred to Dr Gao's "SLAM book" and ORB SLAM design.

Dependencies:
..*OpenCV 3.4
..*G2O
..*DBOW2

Before build,
please change below paths in the code:
1. dataset path in vo.cpp
2. camera parameter in vo.cpp
2. ground truch path in ground_truth_trajectory.cpp

To build:
1. git clone
2. cd build
3. cmake ..
4. make

To Run:
./vo
