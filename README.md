# ros_phantom_intelligence
LiDAR integration for ROS
This package has been tested on Ubuntu 16.04 and 18.04 with, respectively, ROS Kinetic and ROS Melodic.

## Building and Running this package

1. Install the following elements:
 - [ros-"distro"-desktop-full](http://wiki.ros.org/) 
 - [canlib](https://www.kvaser.com/developer/canlib-sdk/) ([linux link](https://www.kvaser.com/linux-drivers-and-sdk/))
 - **python-pip**: 
     ```
     sudo apt-get install python-pip
     ```
 - **run pip command**:
     ```
     pip install --upgrade setuptools
     ``` 
 - [conan](https://conan.io/)
     ```
     pip install conan
     ```
 - [cmake](https://cmake.org/download/) version >= 5 (latest release preferred).
 
 	download and extract the cmake files, open terminal
 	```
 	./bootstrap
 	make
 	sudo make install
 	```
 - **gcc-8 and g++8**
 
    Needed only for ubuntu 16.04.
    ```
    sudo apt-get update
    sudo apt-get install build-essential software-properties-common -y
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
    sudo apt-get install gcc-8 g++-8
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-8
    ```
 - **edit conan profile**:
    ```
    nano ~/.conan/profiles/default
    ```
    Replace its content by the following lines (or add the missing lines):
    ```
    [settings]
    os=Linux
    os_build=Linux
    arch=x86_64
    arch_build=x86_64
    compiler=gcc
    compiler.version=8
    compiler.libcxx=libc++11
    build_type=Release
    [options]
    [build_requires]
    [env]
    ```
2. Create a **ros/catkin** workspace if you don't have one already:  
    > Here, `ros_ws` refers to the **ros/catkin** workspace.   

    ```
    mkdir -p ros_ws/src
    cd ros_ws
    source /opt/ros/melodic/setup.bash
    catkin_make
    source devel/setup.bash
    ```   
3. Clone the [ros_phantom_intelligence](https://github.com/PhantomIntelligence/ros_phantom_intelligence) repo:  
    ```
    cd src
    git clone git@github.com:PhantomIntelligence/ros_phantom_intelligence.git  
    # OR 
    git clone https://github.com/PhantomIntelligence/ros_phantom_intelligence.git
    ```
3. Build it
    ```
    cd ..
    catkin_make 
    ```
4. (Optional) Build and run the tests
    ```
    catkin_make run_tests_phantom_intelligence
    ```
5. (Required to run a gazebo-simulated Lidar)
    ```
    cp -a src/ros_phantom_intelligence/src/lidar_description/. ~/.gazebo/models/
    ```

## Usage with real sensors
### `.launch` file and parameters
The `.launch` files are named after the different sensors.
* `device_location` is the location of the sensor on your machine. The format of the location depends of the sensor you use.  
For `awl` sensors, which uses CAN to communicate, the value represent the **can channel number**.

## Usage with simulated sensors
1. Source the ws after making it.
    ```
    source ~/ros_ws/devel/setup.bash
    ```
2. Launch the launch file.
    ```
    roslaunch phantom_intelligence awl16_gazebo.launch
    ```
The simulated sensor now publishes sensor data under the ros topic named /awl16.

### Example
> Note: Make sure you are in a terminal which has `source <your_ros_ws>/devel/setup.bash` and also make sure that you have built the `ros_phantom_intelligence` package.
1. Find out which sensor model you are using.  
    > `AWL16` will be used for this example.
2. To start the sensor node, run:  
```
roslaunch phantom_intelligence awl16.launch device_location:=0
```  

## Running tests
> Note: Make sure you are in a terminal which has `source <your_ros_ws>/devel/setup.bash` and also make sure that you have built the `ros_phantom_intelligence` package.  
```
roslaunch phantom_intelligence run_tests.launch
```
