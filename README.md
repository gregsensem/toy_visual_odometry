#Visual_Odometry
This is a toy visual odometry built for practice purpose. It extracts ORB features from input 
image frame and do frame by frame feature matching. Then the inlier matched points are found
and used as input for Fundamental / Essential matrix estimation. Camera's pose is recovered with 
the Fundamental / Essentail matrix. The absolute scale is obtained from ground truth.

### Dependencies:
* OpenCV 3.4
* G2O
* DBOW2
* Cmake 3.5

### Before build:
please change below paths in the code:
1. dataset path in vo.cpp
2. camera parameter in vo.cpp
2. ground truch path in ground_truth_trajectory.cpp

### To build:
1. git clone
2. cd build
3. cmake ..
4. make

### To Run:
run ./vo

### example:
<img src="https://github.com/gregsensem/visual_odometry/raw/master/FAST.png" width="50%" height="50%">
<img src="https://github.com/gregsensem/visual_odometry/blob/6af1076a14c48512aac5ca9594db6be5442788bc/running.png" width="50%" height="50%">

### Reference:
1. "14 Lectures' on SLAM", Gao Xiang
2. "ORB SLAM2", Raul Mur-Artal
3. Visual odometry, Avi Sigh
