# ros_phantom_intelligence
LiDAR integration for ROS

## Building and Running this package
1. Install [melodic](http://wiki.ros.org/melodic) 
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

## Usage
### `.launch` file and parameters
The `.launch` files are named after the different sensors.
* `device_location` is the location of the sensor on your machine. The format of the location depends of the sensor you use.  
For `awl` sensors, which uses CAN to communicate, the value represent the **can channel number**.
 

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
