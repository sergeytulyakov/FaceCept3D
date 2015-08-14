# FaceCept3D
FaceCept3D: 3D Face Analysis and Recognition

## Introduction

FaceCept3D is a realtime framework for 3D face analysis and recognition. It contains a set of extendible components that can be combined to fulfil a specific task. At this step we open source the following functionality:

1. Person-specific template creation
2. Extreme head pose estimation [coming soon]
3. Facial expression analysis [coming soon]

FaceCept3D is based on the following works:

* Robust Real-Time Extreme Head Pose Estimation. [[pdf](https://github.com/sergeytulyakov/FaceCept3D/blob/master/papers/Robust%20Real-Time%20Extreme%20Head%20Pose%20Estimation.pdf)]

* Facial expression recognition under a wide range of head poses. [[pdf](https://github.com/sergeytulyakov/FaceCept3D/blob/master/papers/Facial%20Expression%20Recognition%20under%20a%20Wide%20Range%20of%20Head%20Poses.pdf)]

Here is an example image of head pose estimation using FaceCept3D.

![Head Pose Estimation Examples](https://github.com/sergeytulyakov/FaceCept3D/blob/master/home/HalfHeadPoseEstimationExamples.jpg)

See FaceCept3D [wiki](https://github.com/sergeytulyakov/FaceCept3D/wiki) for more details.

## Requirements

To install FaceCept3D you need to have the following libraries available

* [Point Cloud Library PCL](http://www.pointclouds.org/) version 1.7
* [OpenCV](http://opencv.org/) >= version 2.4
* [Qt](http://download.qt.io/archive/qt/4.8/4.8.6/) version 4.8 (required by VTK 5.6)

and their dependencies.

In addition, if you plan to use a depth sensor, you will to install the driver. Currently we support only the Microsoft Kinect 1.0 sensor. However, the code can be easily extended to most of the available RGB-D sensors.

To use MS Kinect sensors:

* On windows install their [SDK (v 1.8)](http://www.microsoft.com/en-us/download/details.aspx?id=40278)

## Installation

FaceCept3D was tested on Windows and Linux. Mac users should follow linux installation instructions.

### Linux

* **OpenNI and SensorKin drivers**. A great guide is [here](https://bitbucket.org/samirmenon/scl-manips-v2/wiki/vision/kinect).

* **Prerequsities**. Install the following libraries using your package manager:

  ```Shell
  boost, eigen, flann, vtk (v 5.6), qhull, opencv, tbb, qt (v 4.8)
  ```

	For Ubuntu 14.04 the following command will do the trick:

	```Shell
	sudo apt-get install libboost-all-dev libeigen3-dev libflann-dev libvtk5-dev libqhull-dev libopencv-dev libtbb-dev libqt4-dev
	```

	In addition, you need to install cmake if you don't have it:

	```Shell
	sudo apt-get install cmake
	```

* **Install PCL**. We need to build and install pcl, since the version [here](http://pointclouds.org/downloads/linux.html) is built without *-std=c++11* modifier. This [guide](http://pointclouds.org/downloads/source.html) will help you build pcl from source.

* **Check that everything works**. Try running pcl_openni_viewier. If you don't see any output, then there is something wrong with your installation.

* **Build FaceCept3D**. 

  ```Shell
  git clone https://github.com/sergeytulyakov/FaceCept3D.git
  cd FaceCept3D
  cmake ..
  make
  ```

### Windows

Windows installation is somewhat more involved, therefore I'll add it a bit later. In meanwhile, feel free to do it by yourself using the provided CMakeLists.txt

### Citing FaceCept3D

If you use FaceCept3D in your research, please cite the one of the following (or both):

	@inproceedings{Tulyakov2014,
		author = {Tulyakov, S. and Vieriu, R. L. and Semeniuta, S. and Sebe, N.},
		booktitle = {Internation Conference on Pattern Recognition},
		title = {{Robust Real-Time Extreme Head Pose Estimation}},
		year = {2014},
	}

	@inproceedings{Vieriu2015,
		author = {Vieriu, R. L. and Tulyakov, S. and Sangineto, E. and Semeniuta, S. and Sebe, N.},
		booktitle = {Face and Gesture Recognition},
		title = {{Facial Expression Recognition under a Wide Range of Head Poses}},
		year = {2015},
	}

