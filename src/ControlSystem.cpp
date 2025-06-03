#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
    E2("enc2"),
    kp(1.0 / dt / 4.6 / 0.7 * 1.0 / dt / 4.6 / 0.7),
    kd(2.0 / dt / 4.6 ),
    M(3441.0 / 104.0 * 3441.0 / 104.0 * 6.8e-8),
    QMax(0.1),
    iInv(104.0 / 3441.0),
    kMInv(1.0 / 8.44e-3),
    R(8.0),
    M1("motor1"),
    // Ex2)
    QdMax(21.2),
    I(3441.0/ 104.0),
    kM(8.44e-3),
    timedomain("Main time domain", dt, true)
{

    // Name all blocks
    E1.setName("E1");   // encoder 1
    E2.setName("E2");   // encoder 2
    e.setName("e");     // control error
    kp.setName("kp");   // Proportional Gain
    ed.setName("ed");   // Error Differntiator
    kd.setName("kd");   // Differntial Gain
    qdd_c.setName("qdd_c"); // correction acceleration
    M.setName("M");         // Mass Matrix
    QMax.setName("QMax");   // Saturation
    iInv.setName("iInv");
    kMInv.setName("kMInv");
    R.setName("R");
    M1.setName("M1");
    // Ex2)
    qd.setName("qd");
    QdMax.setName("QdMax");
    I.setName("i");
    kM.setName("kM");
    U1.setName("U1");

    // Name all signals
    E1.getOut().getSignal().setName("q1 [rad]");
    E2.getOut().getSignal().setName("q2 [rad]");
    e.getOut().getSignal().setName("e [rad]");
    kp.getOut().getSignal().setName("qdd_cp [rad/s^2]");
    ed.getOut().getSignal().setName("ed [rad/s]");
    kd.getOut().getSignal().setName("qdd_cd [rad/s^2]");
    qdd_c.getOut().getSignal().setName("qdd_c [rad/s^2]");
    M.getOut().getSignal().setName("Q [Nm]");
    QMax.getOut().getSignal().setName("Qsat [Nm]");
    iInv.getOut().getSignal().setName("T1 [Nm]");
    kMInv.getOut().getSignal().setName("I1 [A]");
    R.getOut().getSignal().setName("U1q [V]");
    // Motor has no output
    // Ex2)
    qd.getOut().getSignal().setName("qd [rad/s]");
    QdMax.getOut().getSignal().setName("QdSat [rad/s]");
    I.getOut().getSignal().setName("wout [rad/s]");
    kM.getOut().getSignal().setName("U1qd [V]");
    U1.getOut().getSignal().setName("U1 [V]");

    // Connect signals
    e.getIn(0).connect(E2.getOut());
    e.getIn(1).connect(E1.getOut());
    e.negateInput(1);
    kp.getIn().connect(e.getOut());
    ed.getIn().connect(e.getOut());
    kd.getIn().connect(ed.getOut());
    qdd_c.getIn(0).connect(kp.getOut());
    qdd_c.getIn(1).connect(kd.getOut());
    M.getIn().connect(qdd_c.getOut());
    QMax.getIn().connect(M.getOut());
    iInv.getIn().connect(QMax.getOut());
    kMInv.getIn().connect(iInv.getOut());
    R.getIn().connect(kMInv.getOut());
    qd.getIn().connect(E1.getOut());
    QdMax.getIn().connect(qd.getOut());
    I.getIn().connect(QdMax.getOut());
    kM.getIn().connect(I.getOut());
    U1.getIn(0).connect(R.getOut());
    U1.getIn(1).connect(kM.getOut());
    M1.getIn().connect(U1.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(e);
    timedomain.addBlock(kp);
    timedomain.addBlock(ed);
    timedomain.addBlock(kd);
    timedomain.addBlock(qdd_c);
    timedomain.addBlock(M);
    timedomain.addBlock(QMax);
    timedomain.addBlock(iInv);
    timedomain.addBlock(kMInv);
    timedomain.addBlock(R);
    timedomain.addBlock(qd);
    timedomain.addBlock(QdMax);
    timedomain.addBlock(I);
    timedomain.addBlock(kM);
    timedomain.addBlock(U1);
    timedomain.addBlock(M1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}