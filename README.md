This repo contains code that allows you to **launch instances of robots** that are able to **move in the formation** you specify.  
Given code uses **ROS (Robotic Operating System)** as an underlying IPC system. **Gazebo** virtual environment is used for modeling physics and rendering.  
If you launch a file from /scripts/launchers using "gazebo-launch" it will spawn several processes that will find each other on the local host network. Then if Gazebo is launched they will spawn their models inside it.  
After everything has initialized, you can send TCP package with "FORM CIRCLE -x 0.0 -y 0.0 -r 1.0" to any of the units and they will form the shape you specified.  
Features:
* **Automatic connection** and keep-alive status inside the group
* **Collision avoidance**
* Forming geometrical shapes with units
* **Moving in the formation**
* Waiting for slow teammates
* **Reshaping** the order if someone dissapears

(You can actually **run this on Raspberry PI powered robots** if you follow the servo protocol and if you provide them with outer coordinates.)
