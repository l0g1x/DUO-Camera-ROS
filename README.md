DUO-Camera-ROS
==============

ROS driver for DUO Stereo camera(s). Specify which camera you are using in your launch file, if it is a custom DUO solution, specify the the camera specifications in the launch parameters.

There are two different nodes, each with their own respective launch file:
	- duo_node: 		Plain ROS camera driver that interfaces the DUO camera using the optimized DUO SDK/API
		- duo_node.launch: Launches this node with custom user specifications for camera settings
	- duo_dense3d_node: 	Same functionality as duo_node, except also implements the Dense3d DUO Library which processes image disparity (Same thing as stereo_image_proc, except alot more efficient)
		- duo_dense3d_node.launch: Launches everything needed to get a disparity image, based on user specificied parameters for the camera specifications
