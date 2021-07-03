//
// Created by mat on 8/26/17.
//

#ifndef NEUTRON_CONTROLLER_NEURONMOTORDEFINITION_H
#define NEUTRON_CONTROLLER_NEURONMOTORDEFINITION_H

enum NeutronSensorNames{
    NEURON_SENSORAX = 0,
};

enum NeutronMotorNames{
    BC0 = 0,
    CF0 = 1,
    FT0 = 2,

    BC1 = 3,
    CF1 = 4,
    FT1 = 5,

    BC2 = 6,
    CF2 = 7,
    FT2 = 8,

    BC3 = 9,
    CF3 = 10,
    FT3 = 11,

    BC4 = 12,
    CF4 = 13,
    FT4 = 14,

    BC5 = 15,
    CF5 = 16,
    FT5 = 17,

    //Changing according to the maximum motor number
    NEURON_MOTOR_MAX = 18,
};

enum BehaviorNames{
    directionBehavior = 0,
    obstacleBehavior = 1,
    tiltBehavior = 2,

    highBehavior = 3,
    lowBehavior = 4,
    narrowBehavior = 5,

    pipeBehavior = 6,
    flipBehavior = 7,
    wallBehavior = 8,

    walknomedBehavior = 9,
};

#endif //NEUTRON_CONTROLLER_NEURONMOTORDEFINITION_H
