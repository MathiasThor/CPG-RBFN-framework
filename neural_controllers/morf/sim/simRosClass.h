//
// Created by mat on 8/2/17.
//

#ifndef ROS_HEXAPOD_CONTROLLER_SIMROSCLASS_H
#define ROS_HEXAPOD_CONTROLLER_SIMROSCLASS_H

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/ColorRGBA.h"
#include <std_msgs/Int32.h>
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <rosgraph_msgs/Clock.h>
#include "neutronMotorDefinition.h"
#include <random>
#include <iostream>

#include "extApi.h"
#include "extApiPlatform.h"
#include "simConst.h"


class simRosClass {
private:
    // Topics
    std::string GraphTopic;
    std::string MotorTopic;
    std::string jointPositionTopic;
    std::string jointTorqueTopic;
    std::string jointVelocityTopic;
    std::string testParametersTopic;
    std::string nameAddOn;

    // Subscribers
    ros::Subscriber jointPositionSub;
    ros::Subscriber jointTorqueSub;
    ros::Subscriber jointVelocitySub;
    ros::Subscriber testParametersSub;
    ros::Subscriber joySub;
    ros::Subscriber imu_imu;
    ros::Subscriber imu_euler;

    // Publishers
    ros::Publisher MotorPositionPub;
    ros::Publisher GraphPub;

    // Private Methods
    void jointPositionCallback(const std_msgs::Float32MultiArray& jointPositions);
    void jointTorqueCallback(const std_msgs::Float32MultiArray& jointTorques);
    void jointVelocityCallback(const std_msgs::Float32MultiArray& jointVelocities);
    void testParametersCallback(const std_msgs::Float32MultiArray& jointPositions);
    void joy_CB(const sensor_msgs::Joy::ConstPtr& joy);
    void imu_imu_CB(const sensor_msgs::Imu::ConstPtr& imu);
    void imu_euler_CB(const geometry_msgs::Vector3::ConstPtr& euler);

    // Simulation remote API
    int clientID = -1;


public:
    // Public Methods
    simRosClass(int simulationID);
    ~simRosClass();
    void setLegMotorPosition(std::vector<float> positions);
    void setGraph(std::vector<float> graphdata);

    void rosSpinOnce();
    bool shortMorf = true;

    // Public Global Variables
    std::vector<float> testParameters   ={0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::vector<float> jointPositions   ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::vector<float> jointTorques     ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::vector<float> jointVelocities  ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::vector<float> axes             ={0,0,0,0,0,0,0,0};
    std::vector<int>   buttons          ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::vector<float> footForceSensor  ={0,0,0,0,0,0};
    std::vector<float> bboxDim          ={0,0,0};
    std::vector<float> maxBboxDim       ={0,0,0};
    std::vector<float> behaviorSignal   ={0,0,0,0,0};

    float meanHeight            = 100.0;
    float robotCollision        = 100.0;
    float avgPower              = 100.0;
    float avgBodyVel            = 100.0;
    float distance              = 100.0;
    float heightVariance        = 100.0;
    float tiltVariance          = 100.0;
    float headingDirection      = 100.0;
    float tilt                  = 100.0;
    float roll                  = 100.0;
    float slipping              = 100.0;
    float legTouch              = 100.0;
    float walkingDirection      = 100.0;
    float flipRoll              = 100.0;
    float heightDistance        = 100.0;
    float simTime               = 0;
    float terminate             = 100.0;
    std::vector<float> IMU_euler  = {0,0,0};
};


#endif //ROS_HEXAPOD_CONTROLLER_SIMROSCLASS_H
