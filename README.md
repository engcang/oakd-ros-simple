# OAK-D ROS Simple
+ OAK-D (OpenCV AI Kit camera) ROS simple codes with C++ for myself
+ Edited the original `depthai-ros-examples` repo from official manufacturer - [link](https://github.com/luxonis/depthai-ros-examples)

<br>
<br>

## How to install

+ Git clone and install `depthai-core` first

```shell
$ git clone --recursive git@github.com:engcang/oakd-ros-simple
$ cd oakd-ros-simple

$ git clone https://github.com/luxonis/depthai-core
$ cd depthai-core
$ git submodule update --init --recursive

$ sudo apt install libusb-1.0-0-dev

$ mkdir build && cd build
$ cmake .. -DBUILD_SHARED_LIBS=ON
$ make install
```

+ Build this repo

```shell
$ cd ~/your_workspace (check directory)
$ sudo apt install ros-melodic-vision-msgs

Check directory (your_workspace)
$ catkin build -Ddepthai_DIR=<your_workspace>/src/oakd-ros-simple/depthai-core/build/install/lib/cmake/depthai
```
