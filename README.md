# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

# Track an Object in 3D Space

## FP.1 Match 3D Objects

Implemented in camFusion_Student.cpp matchBoundingBoxes function.

## FP.2 Compute Lidar-based TTC

implemented in camFusion_Student.cpp computeTTCLidar function.

Focus ego lane and used median x value to have robust result.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes

Implemented in camFusion_Student.cpp clusterKptMatchesWithROI function.

## Compute Camera-based TTC

Implemented in camFusion_Student.cpp computeTTCCamera function. 

used median value to have robust result.

## FP.5 Performance Evaluation 1

Could not find plausible result from sample lidar data. 

<img src="images/e01.png" /> 
<img src="images/e02.png" /> 
<img src="images/e03.png" />  

<img src="images/l01.png" /> 
<img src="images/l02.png" /> 
<img src="images/l03.png" />   

## FP.6 Performance Evaluation 2

From mid term result tested these methods.

1. FAST-BRIEF (Fast and accuracy)
2. FAST-ORB (Fast less accuracy)
3. BRISK-BRIEF(High accuracy but slower)

Some frame are big difference between Lider and Camera. FAST-BRIEF and FAST-ORB are ok.

| Method        | LIDER           | CAMERA  |
| ------------- |:-------------:| -----:|
| FAST-BRIEF    | 12.13 | 11.91 |
|               | 12.61 | 12.65 |
|               | 14.34 | 21.28 |
|               | 17.05 | 15.29 |
|               | 15.58 | 35.15 |
| FAST-ORB    | 12.13 | 11.53 |
|               | 12.61 | 11.35 |
|               | 14.34 | 16.58 |
|               | 17.05 | 15.66 |
|               | 15.58 | 30.55 |
| BRISK-BRIEF    | 12.13 | 19.52 |
|               | 12.61 | 26.39 |
|               | 14.34 | 13.08 |
|               | 17.05 | 25.58 |
|               | 15.58 | 31.19 |