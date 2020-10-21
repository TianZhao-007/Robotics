- [Robotics](#robotics)
    + [kinematics](#kinematics)
    + [wheel odometry](#wheel-odometry)
    + [Visual Odometry](#visual-odometry)
    + [Kalman filter](#kalman-filter)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


# Robotics
This repo mainly records the process of learning robotics.  
Based on differential drive car, the learning flow starts form kinematics.  
Then,the differential drive car use control algorithm to follow a specific course.  
Next,introducing VO aims to tell the robot where it is.


### kinematics  
* Degree of freedom, fully/under/over-actuated,non-/holonomic  
if DOF > degree of actuation, then it is under-actuated.  
holonomic是完整的约束，non-holonomic是非完整的约束（变分学）  
理解：holonomic是在configuration space上的约束，告诉你哪里不能去。  
non-holonomic是速度上的约束，告诉你哪里不能走，但你依然可以到达那个地方。  

* Pose,rotation matrix  

* free vector and localised vector  

* Notation for oritiation,point and Pose  

* Change of coordinates, change of reference  
  1. change of coordinates: Same vetor but written in different frame.  
  2. Change of reference: Same point written with respect to two different observers.  
  
* Model derivation(foward kinematics)  
  
### wheel odometry  
* noise distribution  
THE noise is like a banana.  

* Derive differential drive robot  
That is, the relation between u,q and wl,wr.  

* Sensors: notation, passive/active, proprioceptive/exteroceptive  

### Visual Odometry   


### Kalman filter  

* possiblity  


### Control knowledge  

### EKF-SLAM  




