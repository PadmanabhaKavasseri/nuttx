#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

class MotorControl {
public:
    MotorControl();
    void sendBLADCMotorMessage(int motor, int on_off, double duty, double frequency);
    void sendSTEPMotorMessage(int motor, int sleep, int direction, double angle=0);
    ~MotorControl();
private:
    bool init();
    struct qrc_pipe_s *m_pipe;

};

#endif  // MOTOR_CONTROL_HPP
