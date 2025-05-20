#ifndef MyRobotSAFETYPROPERTIES_HPP_
#define MyRobotSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "ControlSystem.hpp"

class MyRobotSafetyProperties : public eeros::safety::SafetyProperties
{
public:
    MyRobotSafetyProperties(ControlSystem &cs, double dt);

    // Define all possible events
    // upwards events
    eeros::safety::SafetyEvent doSystemOn;          // blue
    eeros::safety::SafetyEvent systemStarted;       // blue
    eeros::safety::SafetyEvent resetEmergency;      // light green
    eeros::safety::SafetyEvent powerOn;             // violet
    eeros::safety::SafetyEvent startMoving;         // brown
    // downwards events
    eeros::safety::SafetyEvent shutdown;            // purple
    eeros::safety::SafetyEvent motorsHalted;        // yellow
    eeros::safety::SafetyEvent abort;               // red
    eeros::safety::SafetyEvent emergency;           // brown
    eeros::safety::SafetyEvent powerOff;            // light blue
    eeros::safety::SafetyEvent stopMoving;          // green

    // Defina all possible levels
    eeros::safety::SafetyLevel slSystemOff;         // 0
    eeros::safety::SafetyLevel slShuttingDown;      // 1
    eeros::safety::SafetyLevel slBraking;           // 2
    eeros::safety::SafetyLevel slStartingUp;        // 3
    eeros::safety::SafetyLevel slEmergency;         // 4
    eeros::safety::SafetyLevel slEmergencyBraking;  // 5
    eeros::safety::SafetyLevel slSystemOn;          // 6
    eeros::safety::SafetyLevel slMotorPowerOn;      // 7
    eeros::safety::SafetyLevel slSystemMoving;      // 8

private:
    // Define all critical outputs
    eeros::hal::Output<bool>* greenLED;
    eeros::hal::Output<bool>* redLED;

    // Define all critical inputs
    eeros::hal::Input<bool>* buttonPause;
    eeros::hal::Input<bool>* buttonMode;

    ControlSystem &cs;
};

#endif // MyRobotSAFETYPROPERTIES_HPP_
