#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quaternion1"), gain(2.0), signalChecker(-0.2, 0.2),
      timedomain("Main time domain", dt, true) // true = Blocking Sequencer
{
    // Name all blocks
    q1.setName("q1");
    gain.setName("Gain"); // Gain to get alpha
    signalChecker.setName("signalChecker");

    // Name all signals
    q1.getOut().getSignal().setName("alpha/2");
    gain.getOut().getSignal().setName("alpha");

    // Connect signals
    gain.getIn().connect(q1.getOut());
    signalChecker.getIn().connect(gain.getOut());

    // Add blocks to timedomain, has to be in correct sequence order (left to right)
    timedomain.addBlock(q1);
    timedomain.addBlock(gain);
    timedomain.addBlock(signalChecker);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}