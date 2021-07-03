//
// Created by mat on 12/18/19.
//

#ifndef MORF_CONTROLLER_ENVIRONMENT_H
#define MORF_CONTROLLER_ENVIRONMENT_H

#include "extApi.h"
#include "extApiPlatform.h"
#include "simConst.h"
#include <string>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

class environment {
private:
    int clientID = -1;
    int simulationID;
    int internalStepCounter=0;
    int externalStepCounter=0;
    bool simStepDone=false;
    bool useAPItrigger = true;

    ros::Publisher triggerNextStepPub;
    ros::Subscriber simStepDoneSub;
    ros::Subscriber simStepCounterSub;

    void simulationStepDoneCallback(const std_msgs::Bool &_simStepDone);
    void simulationStepCounterCallback(const std_msgs::Int32 &_simStepCounter);
    void synchronousTriggerAPI();
    void synchronousTriggerROS();

public:
    environment(int simulationID, bool _useAPItrigger);
    ~environment();

    void endConnection();
    float getSimulationTime();
    void blackoutSimulation(bool blackout);


    void stop();
    void start();
    void restart();
    void pause();
    void synchronousTrigger();
    void loadScene(char* simulationPath);
    void closeScene();
    void simSetStringSignal(std::string signalName, std::string signalString);
};


#endif //MORF_CONTROLLER_ENVIRONMENT_H
