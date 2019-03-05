# ros_phantom_intelligence
LiDAR integration for ROS

## Building and Running this package

1. Install the following elements:  
 - [melodic](http://wiki.ros.org/melodic) 
 - [canlib](https://www.kvaser.com/developer/canlib-sdk/) ([linux link](https://www.kvaser.com/linux-drivers-and-sdk/))
 - [conan](https://conan.io/)

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
1. Sart the roscore from a terminal.
    ```
    source ~/ros_ws/devel/setup.bash
    roscore
    ```
2. In an other terminal, launch gazebo_ros with the required world file.
    ```
    cd ros_ws/src/ros_phantomintelligence/src/gazebo_driver/worlds
    rosrun gazebo_ros gazebo romanoff.world
    ```
3. In a third terminal:
    ```
    cd ros_ws/devel/lib/phantom_intelligence
    ./gazebo_awl16_node
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
