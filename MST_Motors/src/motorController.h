#ifndef MOTORCONTROLLER_H

enum MOTOR_CONTROLLER_STATUS
{
    MOTOR_ERROR = -1,
    MOTOR_OFF = 0,
    MOTOR_ON,
    MOTOR_MOVING
};

class motorController
{
public:
    motorController(char* device);
    bool    turnOn();
    bool    turnOff();
    int     getState(){return m_status;}
    bool    setVelocityTick(int velocity);
    int     getVelocityTick(){return m_velocity;}
    bool    startMotion();
    bool    stopMotion();

private:
    int         m_fd;
    char        m_name[20];
    int         m_status;
    int         m_velocity;
};

#endif
