DUO-Camera-ROS
==============

This package is a ROS stereo camera driver for DUO cameras. 

The package can act as either a regular camera driver outputing a left and right camera image, or it can act purely as a stereo camera, outputing a disparity image, and a PointCloud2. The user is not limited to one or the other, both left/right camera images can be provided at the same time as disparity and PointCloud2. 

This driver works ONLY with DUO3D (http://duo3d.com/) cameras. 
Utilizing the optimized DUO Dense3D library, eliminates the need for other disparity calculation nodes.  

There are two different nodes:

	- duo_node: 			Generic DUO Camera driver (Left/Right image data)
	- duo_dense3d_node: 	Stereo Camera driver using DUO Dense3D Library
