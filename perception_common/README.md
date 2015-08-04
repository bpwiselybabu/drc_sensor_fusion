README
======

* This library acts as an interface so we can talk to the multisense driver easily. It acts as a one point change of all multisense behaviour in our end.
* It connects to the topics that Wrecs_common describes for multisense autonatically. Using roslaunch parameters it is possible to over write it. All library and code that depend on this library dont have to specify the multisense topics unless they want to overwrite the normal behaviour.

Notes
-----
* It does not subscibe to the topics unless a request has been made. This prevents the ros node to be attached to a topic without needing the information.
* This node also generates a perception_common_drcsim version of the library that compensates for the name and behaviour change in Gazebo simulation. To prevent the perception_common_drcsim libraries to be generated use GAZEBO_SIMULATION=OFF.
* The perception_common_drcsim is not exported in catkin by default. To use it you need to explictly specify it.

