#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quaternion1"), gain(2.0), motorVoltageSetpoint(0.0), motor("motor1"),
      timedomain("Main time domain", dt, true) // true = Blocking Sequencer
{
    // Name all blocks
    q1.setName("q1");
    gain.setName("Gain"); // Gain to get alpha
    motorVoltageSetpoint.setName("motorVoltageSetpoint");
    motor.setName("motor");

    // Name all signals
    q1.getOut().getSignal().setName("alpha/2 [rad]");
    gain.getOut().getSignal().setName("alpha [rad]");
    motorVoltageSetpoint.getOut().getSignal().setName("Motor voltage setpoint [V]");

    // Connect signals
    gain.getIn().connect(q1.getOut());
    motor.getIn().connect(motorVoltageSetpoint.getOut());

    // Add blocks to timedomain, has to be in correct sequence order (left to right)
    timedomain.addBlock(q1);
    timedomain.addBlock(gain);
    timedomain.addBlock(motorVoltageSetpoint);
    timedomain.addBlock(motor);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}