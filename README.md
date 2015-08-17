DUO-Camera-ROS
==============

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/l0g1x/DUO-Camera-ROS?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge)

ROS Driver for DUO3D (http://duo3d.com/) cameras. 

**NOTE:** 

To stay up to date with the latest (required) SDK do the following: 
- `unset DUO_SDK` (unset the enviornment variable if previously set)
- `catkin_make`
- The setup_duo script will then execute
- After it's finished, **it will print the path** to where the new downloaded DUOSDK now is
- Copy the command **from the log**: `export DUO_SDK=/path/to/catkin_ws/devel/DUOSDK`
- Paste it into your `~/.bashrc`



