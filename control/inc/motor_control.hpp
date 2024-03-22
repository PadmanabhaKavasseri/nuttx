#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

class MotorControl {
public:
    MotorControl();
    void sendMotorMessage(int motor, int on_off, double duty, double frequency);
    ~MotorControl();
private:
    bool init();
    struct qrc_pipe_s *m_pipe;




};

#endif  // MOTOR_CONTROL_HPP
