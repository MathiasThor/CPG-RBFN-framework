//
// Created by mat on 12/30/17.
//
//    puts("Path info by use environment variable PWD:");
//    printf("\tWorkdir: %s\n", getenv("PWD"));
//    printf("\tFilepath: %s/%s\n", getenv("PWD"), __FILE__);

#include "neutronController.h"

neutronController::neutronController(int argc,char* argv[]) {
    // initialize random seed
    srand (time(NULL));

    // Controller input
    simulationID        = std::stoi(argv[1]);
    rollout             = std::stoi(argv[2]);
    simulationTime      = std::stoi(argv[3]); // Seconds
    blackOut            = std::stoi(argv[4]); // True or false
    policySelector      = std::stoi(argv[5]); // 1="load trajectory policy", 2="load trajectory and sensory fb policy", 3="load base policy"
    behaviour           = argv[6];                // "walk", "tilt", "direction", "obstacle", "roll"
    simulation          = true;
    rosHandle           = new simRosClass(simulationID);
    env                 = new environment(simulationID, useAPItrigger);
    inputCollector      = new input();

    if(simulation)
        env->simSetStringSignal("behaviour", behaviour);

    if (behaviour == "multiple_primitive"){
        behaviour = "multiple";
        primitive = true;
      }

    // Open data log
    if(useLogData) {
        vector<vector<float>> tmp (9, vector<float>(6, 0));
        logBehaviorData(0, tmp, "open");
    }

    /* DELAY LINE + DIL */
    Delayline tmp(taudil);
    for (int i = 0; i < 18; ++i)
        delayline.push_back(tmp);

    DIL = new dualIntegralLearner(true, 10);

    actual_LPF.reserve(18);
    command_LPF.reserve(18);
    error_per_joint_LPF.reserve(18);

    for (int k = 0; k < 18; ++k) {
        actual_LPF[k] = new lpf;
        actual_LPF[k]->setA(0.90);
        command_LPF[k] = new lpf;
        command_LPF[k]->setA(0.90);
        error_per_joint_LPF[k] = new lpf;
        error_per_joint_LPF[k]->setA(0.996);
    }
    error_LPF = new lpf;
    error_LPF->setA(0.996);
    /* DELAY LINE + DIL*/

    // Set up arrays
    positions.resize(22);
    LPF_1.reserve(6);
    LPF_2.reserve(6);
    LPF_3.reserve(6);
    LPF_4.reserve(6);
    LPF_5.reserve(6);
    LPF_6.reserve(6);
    sensorInputIntegrated.reserve(6);

    // Set up integrator array
    for (int j = 0; j < 6; ++j)
        sensorInputIntegrated[j] = 0;

    // Set up LowPassFilters
    for (int k = 0; k < 6; ++k) {
        LPF_1[k] = new lpf;
        LPF_1[k]->setA(0.95);
        LPF_2[k] = new lpf;
        LPF_2[k]->setA(0.375);  // Direction
        LPF_3[k] = new lpf;
        LPF_4[k] = new lpf;
        LPF_5[k] = new lpf;
        LPF_3[k]->setA(0.5);    // Obstacle 1
        LPF_4[k]->setA(0.5);    // Obstacle 2
        LPF_5[k]->setA(0.95);   // Obstacle 3
        LPF_6[k] = new lpf;
        LPF_6[k]->setA(0.960);
    }

    // Select what we are going to optimize
    // -------------------------------------
    // Load trajectory parameters from RL_job.json (NO FB)
    if(policySelector == 1) {
        if(rollout == -1)
            policyWeights.push_back(readParameterSet("base_controller"));
        else
            policyWeights.push_back(readParameterSet("trajectory"));

        policyWeightsSensor = {};
    // Load trajectory parameters from RL_job_base.json and sensory parameters from RL_job.json (selected by 'behaviour')
    }else if(policySelector == 2){
        policyWeights.push_back(readParameterSet("base_controller"));
        if(behaviour == "multiple") {
            policyWeightsSensor.push_back(readParameterSet("direction")); // Note order is important
            policyWeightsSensor.push_back(readParameterSet("obstacle"));
            policyWeightsSensor.push_back(readParameterSet("tilt"));
            policyWeightsSensor.push_back(readParameterSet("high"));
            policyWeightsSensor.push_back(readParameterSet("low"));
            policyWeightsSensor.push_back(readParameterSet("narrow"));
            policyWeightsSensor.push_back(readParameterSet("pipe"));
            policyWeightsSensor.push_back(readParameterSet("wall"));
            // policyWeightsSensor.push_back(readParameterSet("walknomed"));
        } else if(policySelector == 2 and rollout == -1){
            policyWeightsSensor.push_back(readParameterSet(behaviour));
        } else
            policyWeightsSensor.push_back(readParameterSet("closed_loop"));
    // Load trajectory parameters from RL_job_base.json (NO FB)
    }else if(policySelector == -1){
        policyWeights.push_back(readParameterSet("base_controller"));
        policyWeightsSensor = {};
    }

    // Select between CPG or CPG-RBF network
    if(CPGLearning) {
        CPG = new modularController(1, true);
        CPG->setPhii(startPhi);
    } else {
        CPG_RBFN = new cpg_rbfn(policyWeights, encoding, 20, behaviour, policyWeightsSensor);
        CPG_RBFN->setPhii(startPhi); // FOR THE PAPER THIS IS: 0.015 * M_PI
        CPGPeriodPostprocessor = new postProcessing();
    }

    // Calculate init period and fill all delay line's
    // We use random number to let the CPG start at different time = improved robustness
    int randomNumber = rand() % tau + 1; // Random number between 1 and tau // was = 1
    for (int i = 0; i < tau+randomNumber; ++i) {
        CPGPeriodPostprocessor->calculateAmplitude(CPG_RBFN->getCpgOutput(0), CPG_RBFN->getCpgOutput(1));
        CPGPeriod = CPGPeriodPostprocessor->getPeriod();
        CPG_RBFN->setCPGPeriod(CPGPeriod);

        // If using only trajectory policy else input sensory feedback
        if(abs(policySelector) == 1)
            CPG_RBFN->step();
        else
            CPG_RBFN->step(vector<vector<float>> {}); // rosHandle->footForceSensor);

        vector<double> RBF_output = CPG_RBFN->getNetworkOutput();
    }

    if(simulation) {
        // Start the environment
        env->start();
        // Use black screen if true
        env->blackoutSimulation(blackOut);
        // Use Sync. Trigger between controller and simulation
        env->synchronousTrigger();
    }
}

int neutronController::runController() {
    // While ROS connected
    if(ros::ok()) {
        // Trigger simulation
        if(simulation) {
            if(skipTrigger % 1 == 0)
                env->synchronousTrigger();
            skipTrigger++;
        }

        // Sensory feedback arrays (for each leg)
        vector<vector<float>> sensoryFeedback (3, vector<float>(6, 0));
        vector<vector<float>> postProcessedSensoryFeedback (9, vector<float>(6, 0));
        int behaviour_index = 0;

        // Running one of the CPG-RBF network behaviours
        for (int j = 0; j < 6; ++j) {
            /* Direction behaviour */
            // Walk in the direction specified by rosHandle->walkingDirection
            if((behaviour == "direction" && currentSimTime > 1.5*1000 )|| behaviour == "multiple" ) {
                behaviour_index = 0;

                float directionError = (rosHandle->IMU_euler[2] - rosHandle->walkingDirection);
                postProcessedSensoryFeedback[behaviour_index][j] = LPF_2[j]->step(directionError*1.2);

                if (postProcessedSensoryFeedback[behaviour_index][j] < 0) { // Direction to the left
                    if (j > 2)
                        postProcessedSensoryFeedback[behaviour_index][j] = -abs(postProcessedSensoryFeedback[behaviour_index][j]);
                    else
                        postProcessedSensoryFeedback[behaviour_index][j] = abs(postProcessedSensoryFeedback[behaviour_index][j]);
                } else if (postProcessedSensoryFeedback[behaviour_index][j] > 0) { // Direction to the right
                    if (j <= 2)
                        postProcessedSensoryFeedback[behaviour_index][j] = -abs(postProcessedSensoryFeedback[behaviour_index][j]);
                    else
                        postProcessedSensoryFeedback[behaviour_index][j] = abs(postProcessedSensoryFeedback[behaviour_index][j]);
                }

                // Threshold for max learned turn
                if(postProcessedSensoryFeedback[behaviour_index][j] > 0.5)
                    postProcessedSensoryFeedback[behaviour_index][j] = 0.5;

                if(postProcessedSensoryFeedback[behaviour_index][j] < -0.5)
                    postProcessedSensoryFeedback[behaviour_index][j] = -0.5;

//                postProcessedSensoryFeedback[behaviour_index][j] = 0; // TODO
            }

            /* Obstacle behaviour */
            // Act based on jointTorques. These are currently binary only reacting with the obstacle.
            if(behaviour == "obstacle" || behaviour == "multiple") {
                // Order in 2D array

                if(behaviour == "multiple")
                    behaviour_index = 1;

                sensoryFeedback[behaviour_index][j] = abs(rosHandle->jointTorques[j * 3]);

                if(sensoryFeedback[behaviour_index][j] < 5)
                    postProcessedSensoryFeedback[behaviour_index][j] = 0;
                else {
                    postProcessedSensoryFeedback[behaviour_index][j] = sensoryFeedback[behaviour_index][j];
                }

                postProcessedSensoryFeedback[behaviour_index][j] = LPF_3[j]->step(postProcessedSensoryFeedback[behaviour_index][j]);
                postProcessedSensoryFeedback[behaviour_index][j] = LPF_4[j]->step(postProcessedSensoryFeedback[behaviour_index][j]);
                postProcessedSensoryFeedback[behaviour_index][j] = LPF_5[j]->step(postProcessedSensoryFeedback[behaviour_index][j]);

                if(rosHandle->behaviorSignal[0]!=1 && !primitive)
                    postProcessedSensoryFeedback[behaviour_index][j] = 0; // Inhibit if high is not active
            }

            /* Minimize tilt */
            // Tilt is projected to the legs on the right or left side based on the sign of tilt
            if(behaviour == "tilt" || behaviour == "multiple") {
                // Order in 2D array
                if(behaviour == "multiple")
                    behaviour_index = 2;

                sensoryFeedback[behaviour_index][j] = rosHandle->IMU_euler[1] * 5.0;
                postProcessedSensoryFeedback[behaviour_index][j] = LPF_6[j]->step(sensoryFeedback[behaviour_index][j]);

                if (postProcessedSensoryFeedback[behaviour_index][j] > 0) { // Tilting to the left
                    if (j > 2)
                        postProcessedSensoryFeedback[behaviour_index][j] = 0;
                } else if (postProcessedSensoryFeedback[behaviour_index][j] < 0) { // Tilting to the right
                    if (j <= 2)
                        postProcessedSensoryFeedback[behaviour_index][j] = 0;
                    else
                        postProcessedSensoryFeedback[behaviour_index][j] = abs(postProcessedSensoryFeedback[behaviour_index][j]);
                }

                if(rosHandle->behaviorSignal[0]==1 && !primitive)
                    postProcessedSensoryFeedback[behaviour_index][j] = 0; // Inhibit if high is active
            }
            /* Minimize roll */
            // Roll is projected to the front or middle legs based on the sign of roll (middle legs are kept static)
            if(behaviour == "roll") {
                sensoryFeedback[behaviour_index][j] = rosHandle->IMU_euler[0] * 7.5;
                postProcessedSensoryFeedback[behaviour_index][j] = LPF_4[j]->step(sensoryFeedback[behaviour_index][j]);

                if(postProcessedSensoryFeedback[behaviour_index][j] > 0) { // Rolling to the front
                    if (j != 0 && j != 3)
                        postProcessedSensoryFeedback[behaviour_index][j] = 0;
                } else if(postProcessedSensoryFeedback[behaviour_index][j] < 0) { // Rolling to the hind
                    if (j != 2 && j != 5)
                        postProcessedSensoryFeedback[behaviour_index][j] = 0;
                    else
                        postProcessedSensoryFeedback[behaviour_index][j] = abs(postProcessedSensoryFeedback[behaviour_index][j]);
                }
            }
            if(behaviour == "high" || behaviour == "multiple") {
                if(behaviour == "multiple")
                    behaviour_index = 3;
                postProcessedSensoryFeedback[behaviour_index][j] = rosHandle->behaviorSignal[0];
            }
            if(behaviour == "low" || behaviour == "multiple") {
                if(behaviour == "multiple")
                    behaviour_index = 4;
                postProcessedSensoryFeedback[behaviour_index][j] = rosHandle->behaviorSignal[1];
            }
            if(behaviour == "narrow" || behaviour == "multiple") {
                if(behaviour == "multiple")
                    behaviour_index = 5;
                postProcessedSensoryFeedback[behaviour_index][j] = rosHandle->behaviorSignal[2];
            }
            if(behaviour == "pipe" || behaviour == "multiple") {
                if(behaviour == "multiple")
                    behaviour_index = 6;
                postProcessedSensoryFeedback[behaviour_index][j] = rosHandle->behaviorSignal[3];
            }
            if(behaviour == "wall" || behaviour == "multiple") {
                if(behaviour == "multiple")
                    behaviour_index = 7;
                postProcessedSensoryFeedback[behaviour_index][j] = rosHandle->behaviorSignal[4];
            }
            if(behaviour == "walk") {
                // No sensory feedback is used when learning to walk. This
                // is considered the base behaviour and should work without
                // sensory feedback.
            }
        }

        if(useLogData) {
//            logBehaviorData(rosHandle->simTime, postProcessedSensoryFeedback, "write");
            logBehaviorData(rosHandle->simTime, CPG_RBFN->getContribution(), "write");
        }

        // If only using the CPG and NOT the RBFN
        if(CPGLearning) {
            CPG->step();
            tripodGaitRangeOfMotion(policyWeights, encoding);
        } else { // If using the CPG-RBF network
            // Calculate period of the CPG
            CPGPeriodPostprocessor->calculateAmplitude(CPG_RBFN->getCpgOutput(0), CPG_RBFN->getCpgOutput(1));
            CPGPeriod = CPGPeriodPostprocessor->getPeriod();
            CPG_RBFN->setCPGPeriod(CPGPeriod);

            // Step CPG-RBFN (with (-1/1) or without sensory feedback (2))
            if(abs(policySelector) == 1)
                CPG_RBFN->step();
            else
                CPG_RBFN->step(postProcessedSensoryFeedback);

            // Run the controller and organize for the different joints in 'positions'
            tripodGaitRBFN();
        }

        // Send new controller output positions to the joints
        rosHandle->setLegMotorPosition(positions);
        rosHandle->rosSpinOnce();

        // If sim stopped
        if(rosHandle->terminate < 0) {
            return 0;
        }

    } else {
        // Return is ROS not OK
        return 0;
    }

    // Stop simulation after 'simulationTime' seconds
    if(simulation) {
        currentSimTime = env->getSimulationTime();

        if ( currentSimTime > simulationTime * 1000) {
            if ( rosHandle->slipping == 100.0 ){
                cout << "'";
                return 2;
            } else {
                fitnessLogger();
            }
            return -1;
        }


    }
    return 1;
}

void neutronController::tripodGaitRBFN() {
    vector<double> RBF_output = CPG_RBFN->getNetworkOutput();

    positions.at(BC0) = RBF_output[0];
    positions.at(BC1) = RBF_output[1];
    positions.at(BC2) = RBF_output[2];
    positions.at(BC3) = RBF_output[3];
    positions.at(BC4) = RBF_output[4];
    positions.at(BC5) = RBF_output[5];

    positions.at(CF0) = RBF_output[6];
    positions.at(CF1) = RBF_output[7];
    positions.at(CF2) = RBF_output[8];
    positions.at(CF3) = RBF_output[9];
    positions.at(CF4) = RBF_output[10];
    positions.at(CF5) = RBF_output[11];

    positions.at(FT0) = RBF_output[12];
    positions.at(FT1) = RBF_output[13];
    positions.at(FT2) = RBF_output[14];
    positions.at(FT3) = RBF_output[15];
    positions.at(FT4) = RBF_output[16];
    positions.at(FT5) = RBF_output[17];
}

void neutronController::tripodGaitRangeOfMotion(vector<vector<float>> policyWeights, string encoding) {

    if(encoding == "indirect"){
        positions.at(BC0) = (policyWeights[0][0] * CPG->getCpgOutput(1)  ) + policyWeights[0][1];
        positions.at(BC1) = ( policyWeights[0][0]*-CPG->getCpgOutput(1) ) + policyWeights[0][1];
        positions.at(BC2) = (policyWeights[0][0] * CPG->getCpgOutput(1)  ) + policyWeights[0][1];
        positions.at(BC3) = ( policyWeights[0][0]*-CPG->getCpgOutput(1) ) + policyWeights[0][1];
        positions.at(BC4) = (policyWeights[0][0] * CPG->getCpgOutput(1)  ) + policyWeights[0][1];
        positions.at(BC5) = ( policyWeights[0][0]*-CPG->getCpgOutput(1) ) + policyWeights[0][1];

        positions.at(CF0) = (policyWeights[0][2] * CPG->getCpgOutput(0)  ) + policyWeights[0][3];
        positions.at(CF1) = ( policyWeights[0][2]*-CPG->getCpgOutput(0) ) + policyWeights[0][3];
        positions.at(CF2) = (policyWeights[0][2] * CPG->getCpgOutput(0)  ) + policyWeights[0][3];
        positions.at(CF3) = ( policyWeights[0][2]*-CPG->getCpgOutput(0) ) + policyWeights[0][3];
        positions.at(CF4) = (policyWeights[0][2] * CPG->getCpgOutput(0)  ) + policyWeights[0][3];
        positions.at(CF5) = ( policyWeights[0][2]*-CPG->getCpgOutput(0) ) + policyWeights[0][3];

        positions.at(FT0) = (policyWeights[0][4] * CPG->getCpgOutput(0)  ) + policyWeights[0][5];
        positions.at(FT1) = ( policyWeights[0][4]*-CPG->getCpgOutput(0) ) + policyWeights[0][5];
        positions.at(FT2) = (policyWeights[0][4] * CPG->getCpgOutput(0)  ) + policyWeights[0][5];
        positions.at(FT3) = ( policyWeights[0][4]*-CPG->getCpgOutput(0) ) + policyWeights[0][5];
        positions.at(FT4) = (policyWeights[0][4] * CPG->getCpgOutput(0)  ) + policyWeights[0][5];
        positions.at(FT5) = ( policyWeights[0][4]*-CPG->getCpgOutput(0) ) + policyWeights[0][5];
    }

    if(encoding == "sindirect"){
        positions.at(BC0) = (policyWeights[0][0] * CPG->getCpgOutput(1)  ) + policyWeights[0][1];
        positions.at(BC1) = ( policyWeights[0][2]*-CPG->getCpgOutput(1) ) + policyWeights[0][3];
        positions.at(BC2) = (policyWeights[0][4] * CPG->getCpgOutput(1)  ) + policyWeights[0][5];
        positions.at(BC3) = ( policyWeights[0][0]*-CPG->getCpgOutput(1) ) + policyWeights[0][1];
        positions.at(BC4) = (policyWeights[0][2] * CPG->getCpgOutput(1)  ) + policyWeights[0][3];
        positions.at(BC5) = ( policyWeights[0][4]*-CPG->getCpgOutput(1) ) + policyWeights[0][5];

        positions.at(CF0) = (policyWeights[0][6] * CPG->getCpgOutput(0)  ) + policyWeights[0][7];
        positions.at(CF1) = ( policyWeights[0][8]*-CPG->getCpgOutput(0) ) + policyWeights[0][9];
        positions.at(CF2) = (policyWeights[0][10] * CPG->getCpgOutput(0)  ) + policyWeights[0][11];
        positions.at(CF3) = ( policyWeights[0][6]*-CPG->getCpgOutput(0) ) + policyWeights[0][7];
        positions.at(CF4) = (policyWeights[0][8] * CPG->getCpgOutput(0)  ) + policyWeights[0][9];
        positions.at(CF5) = ( policyWeights[0][10]*-CPG->getCpgOutput(0) ) + policyWeights[0][11];

        positions.at(FT0) = (policyWeights[0][12] * CPG->getCpgOutput(0)  ) + policyWeights[0][13];
        positions.at(FT1) = ( policyWeights[0][14]*-CPG->getCpgOutput(0) ) + policyWeights[0][15];
        positions.at(FT2) = (policyWeights[0][16] * CPG->getCpgOutput(0)  ) + policyWeights[0][17];
        positions.at(FT3) = ( policyWeights[0][12]*-CPG->getCpgOutput(0) ) + policyWeights[0][13];
        positions.at(FT4) = (policyWeights[0][14] * CPG->getCpgOutput(0)  ) + policyWeights[0][15];
        positions.at(FT5) = ( policyWeights[0][16]*-CPG->getCpgOutput(0) ) + policyWeights[0][17];
    }

    if(encoding == "direct"){
        positions.at(BC0) = (policyWeights[0][0] * CPG->getCpgOutput(1)  ) + policyWeights[0][1];
        positions.at(BC1) = ( policyWeights[0][2]*-CPG->getCpgOutput(1) ) + policyWeights[0][3];
        positions.at(BC2) = (policyWeights[0][4] * CPG->getCpgOutput(1)  ) + policyWeights[0][5];
        positions.at(BC3) = ( policyWeights[0][6]*-CPG->getCpgOutput(1) ) + policyWeights[0][7];
        positions.at(BC4) = (policyWeights[0][8] * CPG->getCpgOutput(1)  ) + policyWeights[0][9];
        positions.at(BC5) = ( policyWeights[0][10]*-CPG->getCpgOutput(1) ) + policyWeights[0][11];

        positions.at(CF0) = (policyWeights[0][12] * CPG->getCpgOutput(0)  ) + policyWeights[0][13];
        positions.at(CF1) = ( policyWeights[0][14]*-CPG->getCpgOutput(0) ) + policyWeights[0][15];
        positions.at(CF2) = (policyWeights[0][16] * CPG->getCpgOutput(0)  ) + policyWeights[0][17];
        positions.at(CF3) = ( policyWeights[0][18]*-CPG->getCpgOutput(0) ) + policyWeights[0][19];
        positions.at(CF4) = (policyWeights[0][20] * CPG->getCpgOutput(0)  ) + policyWeights[0][21];
        positions.at(CF5) = ( policyWeights[0][22]*-CPG->getCpgOutput(0) ) + policyWeights[0][23];

        positions.at(FT0) = (policyWeights[0][24] * CPG->getCpgOutput(0)  ) + policyWeights[0][25];
        positions.at(FT1) = ( policyWeights[0][26]*-CPG->getCpgOutput(0) ) + policyWeights[0][27];
        positions.at(FT2) = (policyWeights[0][28] * CPG->getCpgOutput(0)  ) + policyWeights[0][29];
        positions.at(FT3) = ( policyWeights[0][30]*-CPG->getCpgOutput(0) ) + policyWeights[0][31];
        positions.at(FT4) = (policyWeights[0][32] * CPG->getCpgOutput(0)  ) + policyWeights[0][33];
        positions.at(FT5) = ( policyWeights[0][34]*-CPG->getCpgOutput(0) ) + policyWeights[0][35];
    }
}

void neutronController::fitnessLogger() {
    // Average power:  P = T·ω
    // Calculated at each time step
    float avgPower          = rosHandle->avgPower;
    // Average Energy: E = P·t
    // Calculated at each time step
    float avgEnergy         = 0;//avgPower*env->getSimulationTime(); // TODO
    // Minimum distance between colliding parts of the robot
    // Includes intra & inter leg collision and body floor
    float robotColl         = rosHandle->robotCollision;
    // Velocity of the robot body
    // Only for the y-direction/heading direction
    float bodyVelocity      = rosHandle->avgBodyVel;
    // Distance moved
    // In the negative world y-axis/heading direction
    float distance          = rosHandle->distance;
    // Distance moved
    // In the negative world x-axis/upward direction
    float heightDistance    = rosHandle->heightDistance;
    // Height Variance
    // Standard variance for entire run
    float bodyHeightVar     = rosHandle->heightVariance;
    // Panning around z-axis or heading Direction
    // Absolute mean from the entire run (init pan = 0)
    float headingDirection  = rosHandle->headingDirection;
    // Tilting around x-axis
    // Absolute mean from the entire run (init tilt = 0)
    float tilt              = rosHandle->tilt;
    // Rolling around y-axis
    // Absolute mean from the entire run (init roll = 0)
    float roll              = rosHandle->roll;
    // Foot slipping (for all feet)
    // Total amount of slip vs. non slipping ground contact
    float slipping          = rosHandle->slipping;
    // Height Mean
    // Mean for entire run
    float bodyHeightMean    = rosHandle->meanHeight;
    // Leg touching the obstacle Mean
    // Mean for entire run
    float legTouch          = rosHandle->legTouch;
    // Tilt Variance
    // Standard variance for entire run
    float tiltVariance      = rosHandle->tiltVariance;
    // Mean bounding box dimensions
    // Mean for entire run
    vector<float> meanBbox  = rosHandle->bboxDim;
    // Rolling feedback used for flip
    // On the back = 0, standing = 3.14
    float flipRoll          = rosHandle->flipRoll;

    float MORF_weight       = 4.2;  // KG
    float gravity           = 9.82; // m/s^2

    float CoT               = avgEnergy / (MORF_weight * gravity * distance);
    float avgEnergyMeter    = avgEnergy / distance;

    if (isinf(avgEnergyMeter))
        avgEnergyMeter = 0;
    if (isinf(avgEnergy))
        avgEnergy = 0;

    // Weights mean
//        double sumWS = std::accumulate(policyWeightsSensor.begin(), policyWeightsSensor.end(), 0.0);
//        double meanWS = sumWS / policyWeightsSensor.size();

    /* Fitness Sub-Objectives */
    float stability     = 0;
    float collision     = 0;
    float heightError   = 0;
    float headingError  = 0;

    float desiredBodyHeight = 0.025;

    if(behaviour == "walk") {
        // Stay above some height
        if (bodyHeightMean < desiredBodyHeight)
            collision = 1 - (bodyHeightMean / (desiredBodyHeight));
        stability = bodyHeightVar * 120 + headingDirection * 3 + tilt * 4 + roll * 4;
        slipping = slipping * 0.75;
        distance = distance * 3;
        stability = stability * 1;
        collision = collision * 3;
    }else if(behaviour == "walknomed") {
        stability = bodyHeightVar * 2 + headingDirection * 10 + tilt * 1 + roll * 1;
        slipping = slipping * 0.5;
        distance = distance * 3;
        stability = stability * 0.7;
        collision = collision * 3;
    }else if(behaviour == "wall") {
        stability = tilt * 4 + roll * 4;
        slipping = slipping * 0.75;
        distance =  heightDistance * 6 + distance * 0.6;
    }else if(behaviour == "pipe") {
        stability = bodyHeightVar * 10 + headingDirection * 3 + tilt * 16 + roll * 4;
        slipping = slipping * 0.75;
        distance = distance * 5;
        stability = stability * 1;
        collision   = collision * 0;
    }else if(behaviour == "high") {
        stability = bodyHeightVar*10 + headingDirection*3 + tilt*4 + roll*4;
        slipping    = slipping * 1;
        distance    = distance * 3;
        stability   = (stability*10);
        collision   = -meanBbox[2]*15-(meanBbox[1]-0.37)*2;
    }else if(behaviour == "low") {
        // This is for height behaviour
        stability = bodyHeightVar*10 + headingDirection*3 + tilt*4+ roll*4;
        slipping    = slipping * 1;
        distance    = distance * 3;
        stability   = (stability*10);
        collision   = (rosHandle->maxBboxDim[2]-0.16)*60;
    }else if(behaviour == "narrow") { // TODO Cant walk when high or narrow
        stability = bodyHeightVar*10 + headingDirection*3 + tilt*4 + roll*4;
        slipping    = slipping * 1;
        distance    = distance * 3;
        stability   = (stability*10);
//        collision   = -meanBbox[2]*60;
        collision   = (rosHandle->maxBboxDim[1]-0.37)*60;// - (meanBbox[2]-0.16)*70;
    }else if(behaviour == "flip") {
        if(flipRoll > flipRollMax)
            flipRollMax = flipRoll;

        slipping    = 0;
        distance    = flipRollMax;
        stability   = 0;
        collision   = 0;
    }else if(behaviour == "obstacle") {
        stability = bodyHeightVar * 1 + headingDirection * 1 + tilt * 1 + roll * 1;
        slipping = slipping * 0.5;
        distance = distance * 0.5;
        stability = stability * 1;
        collision = legTouch;
    }else if(behaviour == "obstacle_direct") {
        stability = bodyHeightVar*1 + headingDirection*1 + tilt*1 + roll*1;
        slipping    = slipping  * 0.5;
        distance    = distance  * 3;
        stability   = stability * 1;
        collision   = 0;//legTouch;
    }else if(behaviour == "tilt") {
        stability = bodyHeightVar*1 + headingDirection*8 + roll*1;
        slipping    = slipping  * 0.5;
        distance    = distance  * 2;
        stability   = stability * 1;
        collision   = (tilt * 40) + (tiltVariance * 10); // Straight
    }else if(behaviour == "roll") {
        stability = headingDirection*2 + tilt*2;
        collision = pow(robotColl,20);
        slipping    = slipping  * 1;
        distance    = distance  * 4;
        stability   = stability * 1 + collision * 2.5;
        collision   = (roll * 20);// + (tiltVariance * 1); // Straight
    }else if(behaviour == "direction") {
        headingError = (rosHandle->IMU_euler[2] - rosHandle->walkingDirection);
        slipping = slipping * 1;
        distance = distance * 0.1; // a bit high
        stability = bodyHeightVar * 30 + roll * 10 + tilt * 10;
        collision = abs(headingError) * 6;
        // Stay above some height
        if(bodyHeightMean < desiredBodyHeight)
            collision += (1 - (bodyHeightMean / (desiredBodyHeight))) * 3;
    }

    // Todo Make individual
    if (stability > 8)
        stability = 8;

    /* Fitness Function */
    float fitnessValue = (distance) - (stability + collision + slipping);

    // Power implementation not working yet
    float power = 0; //meanWS * 5;

    // Write json file with fitness
    rapidjson::Document document;
    document.SetObject();
    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

    // ADD SUB-FITNESS FUNCTIONS HERE AND INCLUDE THEM IN RL_master.py
    document.AddMember("FitnessValue", fitnessValue, allocator);
    document.AddMember("Fitness_Stab", stability, allocator);
    document.AddMember("Fitness_Coll", collision, allocator);
    document.AddMember("Fitness_Powr", power, allocator);
    document.AddMember("Fitness_Dist", distance, allocator);
    document.AddMember("Fitness_Slip", slipping, allocator);
    document.AddMember("Distance", distance, allocator);
    document.AddMember("Energy", power, allocator);

    // Write to json file
    string jsonfilename = "./../data/jobs/answers/answer_" + std::to_string(rollout) +".json";
    ofstream ofs(jsonfilename);
    rapidjson::OStreamWrapper osw(ofs);
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
    document.Accept(writer);

    writer.Flush();
    usleep(100);

    // Make sure that the file writer is complete else post error msg
    if (!writer.IsComplete()) {
        cout << "[ ERROR] In write JSON - rm answer_" + std::to_string(rollout) << endl;
        if(simulation)
            cout << rosHandle->slipping << " + " << env->getSimulationTime() << endl;
    }
}

vector<float> neutronController::readParameterSet(string parametertype) {
    string job_path = "";
    string parameter_name = "";
    string noise_name = "";
    string prefix = ".";

    if(simulation)
        prefix = "./../data";
    else
        prefix = "/home/morf-one/";

    if(parametertype == "trajectory") {
        if(rollout < 0) {
            std::string rollout_num = std::to_string(abs(rollout));
            job_path = prefix + "/jobs/RL_job_" + rollout_num + ".json";
        } else {
            job_path = prefix + "/jobs/RL_job.json";
        }
        parameter_name = "ParameterSet";
        noise_name = "noise_";
    } else if(parametertype == "closed_loop") {
        if(rollout < 0) {
            std::string rollout_num = std::to_string(abs(rollout));
            job_path = prefix + "/jobs/RL_job_" + rollout_num + ".json";
        } else {
            job_path = prefix + "/jobs/RL_job.json";
        }
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "base_controller") {
        job_path = prefix+"/RL_job_base.json";
        parameter_name = "ParameterSet";
        noise_name = "noise_";
    } else if(parametertype == "direction") {
        job_path = prefix+"/RL_job_direction.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "obstacle") {
        job_path = prefix+"/RL_job_obstacle.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "tilt") {
        job_path = prefix+"/RL_job_tilt.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "roll") {
        job_path = prefix+"/RL_job_roll.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "high") {
        job_path = prefix+"/RL_job_high.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "low") {
        job_path = prefix+"/RL_job_low.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "pipe") {
        job_path = prefix+"/RL_job_pipe.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "wall") {
        job_path = prefix+"/RL_job_wall.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "walknomed") {
        job_path = prefix+"/RL_job_walknomed.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "narrow") {
        job_path = prefix+"/RL_job_narrow.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "flip") {
        job_path = prefix+"/RL_job_flip.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    } else if(parametertype == "obstacle_direct") {
        job_path = prefix+"/RL_job_obstacle_direct.json";
        parameter_name = "SensorParameterSet";
        noise_name = "noise_sensor_";
    }

    ifstream ifs(job_path);
    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document document;
    document.ParseStream(isw);

    assert(document.IsObject());
    rapidjson::Document::AllocatorType &allocator = document.GetAllocator();

    assert(document.HasMember(parameter_name.c_str()));
    assert(document[parameter_name.c_str()].IsArray());
    rapidjson::Value &paramset = document[parameter_name.c_str()];

    vector<float> policyWeights;
    if (rollout >= 0) {

        std::string curr_noise_name = noise_name + std::to_string(rollout); //rollout not set in time!

        if (!document.HasMember(curr_noise_name.c_str())) {
            cout << "[ERROR] No noise member called: " << curr_noise_name.c_str() << endl;
            cout << "[ERROR] Using noise member \"" + noise_name + "0\" again" << endl;
            curr_noise_name = noise_name + "0";
        }

        rapidjson::Value &noise = document[curr_noise_name.c_str()];

        for (rapidjson::SizeType i = 0; i < paramset.Size(); i++) // Uses SizeType instead of size_t
            policyWeights.push_back(paramset[i].GetDouble() + noise[i].GetDouble());
    } else {
        for (rapidjson::SizeType i = 0; i < paramset.Size(); i++) // Uses SizeType instead of size_t
            policyWeights.push_back(paramset[i].GetDouble());
    }

    // Get encoding
    assert(document.HasMember("checked"));
    assert(document["checked"].IsString());
    rapidjson::Value &_encoding = document["checked"];
    encoding = _encoding.GetString();

    ifs.close();

    return policyWeights;
}

void neutronController::logData( double sensor , const string& option){
    if(option == "open"){
        string path = "./../data/RL_log_joint_sensor.txt";
        myFile.open(path);
        myFile << "iteration" << "\t";
        for (int j = 0; j < 18; ++j)
            myFile << "J-" << j << "\t";
        myFile << "Sensor" << "\n";
    } else if (option == "write") {
        myFile << iteration << "\t";
        for (int j = 0; j < 18; ++j)
            //myFile << positions.at(j) << "\t";
            myFile << delayline[j].Read(2) << "\t";
        myFile << sensor << "\n";

        iteration++;
    } else if (option == "close") {
        myFile.close();
    }
}

void neutronController::logBehaviorData(double sim_time, vector<vector<float>> postProcessedSensoryFeedback, const string& option){
    if(option == "open"){

        auto time = std::time(nullptr);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
        auto s = ss.str();
        std::replace(s.begin(), s.end(), ':', '-');

        string path = "./../data/Behavior_data_"+s+".txt";
        myFile.open(path);

        // DIL data
        myFile << "sim_time" << "\t";

        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < 6; ++j)
                myFile << "behavior_" << i << "leg_" << j << "\t";

        myFile << "\n";

    } else if (option == "write") {
        myFile << sim_time << "\t";

        if(sim_time > 0.5) {
            for (int i = 0; i < 9; ++i)
                for (int j = 0; j < 6; ++j) {
                    myFile << postProcessedSensoryFeedback[i][j] << "\t";
                }

            myFile << "\n";
        } else
            for (int i = 0; i < 9; ++i)
                for (int j = 0; j < 6; ++j) {
                    myFile << 0 << "\t";
                }

        myFile << "\n";
    } else if (option == "close") {
        myFile.close();
    }
}

double neutronController::rescale(double oldMax, double oldMin, double newMax, double newMin, double parameter){
    return (((newMax-newMin)*(parameter-oldMin))/(oldMax-oldMin))+newMin;
}

vector<float> neutronController::jointLimiterMORF(vector<float> jointValues){
    jointValues.at(BC0)=jointLimiter(jointValues.at(BC0), -110, 22);
    jointValues.at(CF0)=jointLimiter(jointValues.at(CF0), -5, 130);
    jointValues.at(FT0)=jointLimiter(jointValues.at(FT0), -80, 0);

    jointValues.at(BC1)=jointLimiter(jointValues.at(BC1), -22, 22);
    jointValues.at(CF1)=jointLimiter(jointValues.at(CF1), -5, 130);
    jointValues.at(FT1)=jointLimiter(jointValues.at(FT1), -80, 0);

    jointValues.at(BC2)=jointLimiter(jointValues.at(BC2), -22, 110);
    jointValues.at(CF2)=jointLimiter(jointValues.at(CF2), -5, 130);
    jointValues.at(FT2)=jointLimiter(jointValues.at(FT2), -80, 0);

    jointValues.at(BC3)=jointLimiter(jointValues.at(BC3), -22, 110);
    jointValues.at(CF3)=jointLimiter(jointValues.at(CF3), -5, 130);
    jointValues.at(FT3)=jointLimiter(jointValues.at(FT3), -80, 0);

    jointValues.at(BC4)=jointLimiter(jointValues.at(BC4), -22, 22);
    jointValues.at(CF4)=jointLimiter(jointValues.at(CF4), -5, 130);
    jointValues.at(FT4)=jointLimiter(jointValues.at(FT4), -80, 0);

    jointValues.at(BC5)=jointLimiter(jointValues.at(BC5), -110, 22);
    jointValues.at(CF5)=jointLimiter(jointValues.at(CF5), -5, 130);
    jointValues.at(FT5)=jointLimiter(jointValues.at(FT5), -80, 0);

    return jointValues;
}

float neutronController::jointLimiter(float jointValue, float jointMin, float jointMax) {
    float toRad = M_PI / 180;
    jointMin = jointMin * toRad;
    jointMax = jointMax * toRad;

    if(jointValue > jointMax)
        return jointMax;
    else if(jointValue < jointMin)
        return jointMin;
    else return jointValue;
}

neutronController::~neutronController() {
    // ----------
    //  CLEAN UP
    // ----------
    if(CPGLearning)
        delete CPG;
    else
        delete CPG_RBFN;
    delete CPGPeriodPostprocessor;
    if(simulation) {
        env->stop();
        env->endConnection();
        delete env;
    }
    ros::shutdown();
    delete rosHandle;
    if(useLogData) {
        vector<vector<float>> tmp (9, vector<float>(6, 0));
        logBehaviorData(0, tmp, "close");
    }
}
