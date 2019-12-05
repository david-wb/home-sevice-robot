Home Service Robot

This project navigates a Turtlebot2 in a small simulated environment using the using the Turtlebot AMCL package. 

# System Requirements

Ubuntu 16.04 and ROS Kinetic

# How to Run

Clone the repo and inside the repo folder run

```
rosdep install --from-paths src --ignore-src -r -y
./home-service.sh
```

This will launch gazebo with Turtlebot2 inside the simulated world. It will also launch rviz where you should be able to see the turtlebot navigate to the marker pickup location, pause for 5 seconds, and then navigate to the dropoff location.

Here is a short video of the navigating robot:

![home-service-robot](https://user-images.githubusercontent.com/1855225/70272729-fc490100-1765-11ea-871d-ed0fad8ba037.gif)

