#2026-11-08 @ 11:33 

## 1.0 Proposal for System Architecture:

### 1.1 Generic Setup: 

  #HardwareInterface:
    #RealsenseSDK2              - gives 2D and 3D image- and pc2-streams, needed for target detection and obstacle avoidance
    #DynamixelHardwareInterface - gives states of joints and actuators, and receives effort,velocity or position commands

  #RobotStatePublisher:
    #Robot Model Feature: Kinematic model for optimized motion planning. 
    #Robot Model Feature: Geometric model of arm in euclidean space (planning scene)
    #Robot Model Feature: Bounded extents of arm for collision checking. 

  #RobotControllers: 
    #ControllerSpawner
    #ControllerManager 

  #Mapping? Octomap ?: 
  This is normally a must have, but I'm not sure we will need it for this task. It doesn't hurt to have it running, it's extremely lightweight for such a small ROI. 
  It is a 3D-probabalistic mapping framework, which in addition to obstacles(nodes) also keeps track of unknowns - not to mention cleared points. Coupled with EDT3D (Euclidean Distance Transforms 3D), 
  and you have the best practice collision checking and trajectory optimization only a callback away. 

  
  #Next layer becomes RobotState, where current actuator feedback from the dynamixel bus gives very accurate transforms between every frame of the robot description file.
  This makes filtering local data in the global frame just as easy as expanding the global model based on local perception. With perception sensors mounted on the robot, this becomes more non-optional. 

  The most important transform feature here, is the transformations that take place when our perception-analyzer "BlobDetector" finds a ball and relays it to the operator as a target. 
  Now that we have A and B, our remaining task becomes reaching B in the shortest time, with the least risk and with appropriate force to knock the ball off it's stand.  
    
  Most relevant, however, is the returned output commands for the actuators, created by ROS2-control. This is basically wrappers of every control theory theorem known to man, form PID's and ackerman steering to 
  to model predivative controllers and optimized trajectories.   

  To close the loop, we need this many-DOF'ed move_group that is environment- and self-aware to perform the planning, checking and execution of Blob-targeted goalstate.
  In  many ways, this is the by far most challenging task - programmatically speaking.But as long as the moveit2-configuration is as correct as the camera intrinistics and their respective transform, 
  it should be a 10 in 10 times successs to relay a point within the robots reachable and observable state space. 

  
  ROS2-control can only take us so far; moving single actuators is as tedious as it is error-prone. 

  This brings us to the best practice of implementing trajectory control in the industrial context; namely Moveit2. Moveit provides the most tested, proven and maintained metapackage available for
  complex planning in three dimensions - not only actuating, but also manipulating the world around the robot. This is what every modern manufacturing arm, from china to USA, runs under the hood. 

      
  remove/merge one: *(Internal to moveit2 both during planning and during setup/self-collision/SRDF-generation)
  remove/merge one:  especially moveit configure SRDF construction (self-colliison precalc for faster realtime processing
  remove/merge one:  Geometric / Euclidean representation of every links physical features, for visualizing, planning, collision checking and computations.

  #ROS2_control
  
  #Realsense2 ROS2-wrapper
  #BlobDetector working on either 2D, 3D or both images. Maybe prefer to use realsense's alignment of color to depth, using colored pointcloud2 to find the blobs
  
    #TF2 in a key role wrt path planning and efficiency

  #Octomap
  #EDTO3d
  
  ce wrt kinematics and robot arm motion planning through moveit2.,
#### Launch Files 
#Launch files 

###
