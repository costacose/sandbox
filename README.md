# Learning how to use Docker

## This example shows how to use ROS noetic via Docker Compose (OS agnostic)


Step1:
Structure the repo as in the following:
``` 
- your_repo/
  - docker/
    - Dockerfile
    - docker-compose.yml
    - (other Docker-related files)
  - src/
    - (ROS packages and related files)
  - README.md
  - (other repository-related files)
```
More specifically, this is how it will look initially in this case:
```
- your_repo/
  - docker/
    - docker-compose.yml
  - src/
  - README.md
```
First step is to create the directory structure needed, and this is:

```
cd your_repo/src
mkdir -p test_pkg/scripts
```

Let's also add the python script:
```
touch test_pkg/scripts/hello_transmitter.py
```

Let's add the CMakeLists.txt and package.xml to the test_pkg folder. This is how they look like:

- CMakelist.txt:
```
cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/hello_transmitter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
- package.xml
```
<?xml version="1.0"?>
<package format="2">
  <name>test_pkg</name>
  <version>0.1.0</version>
  <description>A simple ROS package for hello world</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- Other package-specific information -->
</package>

```

Ok, now it's time to work on the docker-compose.yaml.
This is how the file should look like:
```
#docker-compose.yaml
version: '3'

services:
  ros_container:  
    image: ros:noetic
    container_name: my_ros_container
    volumes:
      - ../src:/root/catkin_ws/src  # Mount local 'src' directory to container's '/root/catkin_ws/src'
    command:
      - /bin/bash
      - -c
      - |
        source /opt/ros/noetic/setup.bash && \
        cd /root/catkin_ws/src/test_pkg/scripts && \
        chmod +x hello_transmitter.py && \
        cd /root/catkin_ws && \
        source /opt/ros/noetic/setup.bash && \
        catkin_make && \
        source devel/setup.bash && \
        roslaunch test_pkg test_pkg_launchfile.launch
```

What happens in the docker compose: 

- Docker pulls the ros:noetic image publicly available on the docker hub.
- the `src` folder available on the repo will be mounted in the catkin_ws available in the docker image.
- We feed a sequence of command in the docker image as if we were running ros from an Ubuntu machine (it is still not clear to me how to avoid the catkin_make build everytime that the docker-compose starts).

The easiest way that I could find to start the roscore and the node sequentially is to use a launch file (probably other ways exist but this looked like the easiest way to go).

To launch the docker-compose, type in the `docker` directory:

```
docker-compose up
```
This should start the "hello world" node that keeps transmitting the same message over and over.
If you want to visualize the debug output on the terminal, in a second terminal panel type
```
docker exec -it my_ros_container /bin/bas
```
This will access the running Docker container interactively via a bash shell.

When done, type the following to close the container:
```
docker-compose down
```



