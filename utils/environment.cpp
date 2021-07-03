//
// Created by mat on 12/18/19.
//

#include "environment.h"

environment::environment(int _simulationID, bool _useAPItrigger) {
    // Save simulation ID and trigger method
    simulationID = _simulationID;
    useAPItrigger = _useAPItrigger;

    // Create connection
    simxFinish(-1);
    int connectionPort = 19998 - simulationID;
    clientID = simxStart("127.0.0.1", connectionPort, true, true, 5000, 0);

    // Enable Sync mode
    simxSynchronous(clientID, useAPItrigger);

    // Set simulation ID
    simxSetIntegerSignal(clientID, "simulationID", simulationID, simx_opmode_blocking);

    // ROS
    if(!useAPItrigger) {
        int _argc = 0;
        char **_argv = nullptr;
        std::string nodeName("simulation_controller");
        nodeName += simulationID;
        ros::init(_argc, _argv, nodeName);

        if (!ros::master::check())
            ROS_ERROR("ros::master::check() did not pass!");

        ros::NodeHandle node("~");
        //  ROS_INFO("simROS just started!");
        triggerNextStepPub = node.advertise<std_msgs::Bool>(
                "/sim_control" + std::to_string(simulationID) + "/triggerNextStep", 1);
        simStepDoneSub = node.subscribe("/sim_control" + std::to_string(simulationID) + "/simulationStepDone", 1,
                                        &environment::simulationStepDoneCallback, this);
        simStepCounterSub = node.subscribe("/sim_control" + std::to_string(simulationID) + "/simulationStepCounter", 1,
                                           &environment::simulationStepCounterCallback, this);
    }
}

void environment::simSetStringSignal(std::string signalName, std::string signalString){
    const char *signalName_c = signalName.c_str();
    unsigned char* signalString_c = (unsigned char*) signalString.c_str(); // cast from string to unsigned char*

    simxSetStringSignal(clientID, signalName_c, signalString_c, signalString.length(), simx_opmode_blocking);
}

void environment::simulationStepDoneCallback(const std_msgs::Bool& _simStepDone){
    simStepDone=_simStepDone.data;
}

void environment::simulationStepCounterCallback(const std_msgs::Int32& _simStepCount){
    externalStepCounter=_simStepCount.data;
}

float environment::getSimulationTime() {
    int * pingTime;
    simxGetPingTime(clientID,pingTime);
    return simxGetLastCmdTime(clientID);
}

void environment::synchronousTriggerAPI() {
    simxSynchronousTrigger(clientID);
}

void environment::synchronousTrigger() {
    if(useAPItrigger)
        synchronousTriggerAPI();
    else
        synchronousTriggerROS();
}

void environment::synchronousTriggerROS() {
    std_msgs::Bool _bool;

    _bool.data = true;
    triggerNextStepPub.publish(_bool);
    simStepDone = false;

    float simTime = getSimulationTime();

    while(!simStepDone) {
        ros::spinOnce();
        if(simTime == 0)
            triggerNextStepPub.publish(_bool);
    }
}

void environment::loadScene(char* simulationPath) {
    simxLoadScene(clientID, simulationPath, 0, simx_opmode_blocking);
}

void environment::closeScene() {
    simxCloseScene(clientID,simx_opmode_blocking);
}

void environment::stop() {
    simxStopSimulation(clientID, simx_opmode_blocking);
}

void environment::pause() {
    simxPauseSimulation(clientID, simx_opmode_blocking);
}

void environment::start() {
    simxStartSimulation(clientID, simx_opmode_oneshot);
}

void environment::restart() {
    simxStopSimulation(clientID, simx_opmode_blocking);
    simxStartSimulation(clientID, simx_opmode_oneshot);
}


void environment::endConnection() {
    simxFinish(clientID);
    if(!useAPItrigger)
        ros::shutdown();
}

void environment::blackoutSimulation(bool blackout) {
    simxSetBooleanParameter(clientID, 16, !blackout, simx_opmode_blocking);
}

environment::~environment() {
}
