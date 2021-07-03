//
// Created by mat on 8/2/17.
//

#include "simRosClass.h"

simRosClass::simRosClass(int simulationID){

    std::string stringSimulationID = std::to_string(simulationID);
    std::string simControlName = "/morf_sim" + stringSimulationID;

    GraphTopic=simControlName+"/graph";
    MotorTopic=simControlName+"/multi_joint_command";
    jointPositionTopic=simControlName+"/joint_positions";
    jointTorqueTopic=simControlName+"/joint_torques";
    jointVelocityTopic=simControlName+"/joint_velocities";
    testParametersTopic=simControlName+"/testParameters";

    // Create a ROS node.
    int _argc = 0;
    char** _argv = nullptr;
    std::string nodeName("morf_controller");
    nodeName+=stringSimulationID;
    ros::init(_argc,_argv,nodeName);

    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");

    ros::NodeHandle node("~");
    // ROS_INFO("simROS just started!");

    // Subscribe to topics and specify callback functions
    jointPositionSub=node.subscribe(jointPositionTopic, 1, &simRosClass::jointPositionCallback, this);
    jointTorqueSub=node.subscribe(jointTorqueTopic, 1, &simRosClass::jointTorqueCallback, this);
    jointVelocitySub=node.subscribe(jointVelocityTopic, 1, &simRosClass::jointVelocityCallback, this);
    testParametersSub=node.subscribe(testParametersTopic, 1, &simRosClass::testParametersCallback, this);
    joySub=node.subscribe("/joy"+stringSimulationID, 1, &simRosClass::joy_CB, this);
    imu_imu=node.subscribe("/morf_sim/imu"+stringSimulationID, 1, &simRosClass::imu_imu_CB, this);
    imu_euler=node.subscribe("/morf_sim"+stringSimulationID+"/euler", 1, &simRosClass::imu_euler_CB, this);

    // Initialize publishers
    MotorPositionPub=node.advertise<std_msgs::Float32MultiArray>(MotorTopic,1);
    GraphPub=node.advertise<std_msgs::Float32MultiArray>(GraphTopic,1);

    //rate = new ros::Rate(17*4); // 60hz
}

void simRosClass::jointTorqueCallback(const std_msgs::Float32MultiArray& _jointTorques){
    jointTorques = _jointTorques.data;
}

void simRosClass::joy_CB(const sensor_msgs::Joy::ConstPtr& joy){
    axes = joy->axes;
    buttons = joy->buttons;
}

void simRosClass::imu_imu_CB(const sensor_msgs::Imu::ConstPtr &imu) {
    // To be implemented
}

void simRosClass::imu_euler_CB(const geometry_msgs::Vector3::ConstPtr &euler) {
    // To be implemented
}

void simRosClass::jointVelocityCallback(const std_msgs::Float32MultiArray& _jointVelocities){
    jointVelocities = _jointVelocities.data;
}

void simRosClass::testParametersCallback(const std_msgs::Float32MultiArray& _testParameters) {
    testParameters = _testParameters.data;
    slipping            = testParameters[1];
    tilt                = testParameters[2];
    roll                = testParameters[3];
    headingDirection    = testParameters[4];
    robotCollision      = testParameters[5];
    meanHeight          = testParameters[6];
    avgPower            = testParameters[7];
    avgBodyVel          = testParameters[8];
    distance            = testParameters[9];
    heightVariance      = testParameters[10];
    footForceSensor[0]  = testParameters[11];
    footForceSensor[1]  = testParameters[12];
    footForceSensor[2]  = testParameters[13];
    footForceSensor[3]  = testParameters[14];
    footForceSensor[4]  = testParameters[15];
    footForceSensor[5]  = testParameters[16];
    legTouch            = testParameters[17];
    IMU_euler[0]        = testParameters[18];
    IMU_euler[1]        = testParameters[19];
    IMU_euler[2]        = testParameters[20];
    tiltVariance        = testParameters[21];
    walkingDirection    = testParameters[22];
    bboxDim[0]          = testParameters[23];
    bboxDim[1]          = testParameters[24];
    bboxDim[2]          = testParameters[25];
    flipRoll            = testParameters[26];
    heightDistance      = testParameters[27];
    simTime             = testParameters[28];
    terminate           = testParameters[29];
    maxBboxDim[0]       = testParameters[30];
    maxBboxDim[1]       = testParameters[31];
    maxBboxDim[2]       = testParameters[32];
    behaviorSignal[0]   = testParameters[33];
    behaviorSignal[1]   = testParameters[34];
    behaviorSignal[2]   = testParameters[35];
    behaviorSignal[3]   = testParameters[36];
    behaviorSignal[4]   = testParameters[37];
    behaviorSignal[5]   = testParameters[38];
}

void simRosClass::setLegMotorPosition(std::vector<float> positions) {
    // publish the motor positions:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    if(!shortMorf) {
        positions[CF0] -= 1.57079633; // 90 Degree
        positions[CF1] -= 1.57079633; // 90 Degree
        positions[CF2] -= 1.57079633; // 90 Degree
        positions[CF3] -= 1.57079633; // 90 Degree
        positions[CF4] -= 1.57079633; // 90 Degree
        positions[CF5] -= 1.57079633; // 90 Degree
    }

    positions[BC3] *= -1;
    positions[BC4] *= -1;
    positions[BC5] *= -1;

    std::vector<float> positionsNew = {11,positions.at(BC0),12,positions.at(CF0),13,positions.at(FT0),
                                       21,positions.at(BC1),22,positions.at(CF1),23,positions.at(FT1),
                                       31,positions.at(BC2),32,positions.at(CF2),33,positions.at(FT2),
                                       41,positions.at(BC3),42,positions.at(CF3),43,positions.at(FT3),
                                       51,positions.at(BC4),52,positions.at(CF4),53,positions.at(FT4),
                                       61,positions.at(BC5),62,positions.at(CF5),63,positions.at(FT5)};

    for (float positionsNew : positionsNew)
        array.data.push_back(positionsNew);


    MotorPositionPub.publish(array);
}

void simRosClass::setGraph(std::vector<float> graphData) {
    // publish the motor positions:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < graphData.size(); ++i) {
        array.data.push_back(graphData[i]);
    }

    GraphPub.publish(array);
}

void simRosClass::jointPositionCallback(const std_msgs::Float32MultiArray& _jointPositions){
    jointPositions = _jointPositions.data;

    if(!shortMorf) {
        jointPositions[CF0] += 1.57079633; // 90 Degree
        jointPositions[CF1] += 1.57079633; // 90 Degree
        jointPositions[CF2] += 1.57079633; // 90 Degree
        jointPositions[CF3] += 1.57079633; // 90 Degree
        jointPositions[CF4] += 1.57079633; // 90 Degree
        jointPositions[CF5] += 1.57079633; // 90 Degree
    }

    jointPositions[BC3] *= -1;
    jointPositions[BC4] *= -1;
    jointPositions[BC5] *= -1;
}

void simRosClass::rosSpinOnce(){
    ros::spinOnce();
}

simRosClass::~simRosClass() {
    ros::shutdown();
}
