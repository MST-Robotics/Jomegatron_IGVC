#include "motorController.h"
#include <unistd.h>

#define SPEED 150000
#define TURN_TIME  5
#define FORW_TIME  2

int main()
{

motorController m[] = {motorController("/dev/ttyUSB1"),motorController("/dev/ttyUSB2")};

m[0].turnOn();
m[1].turnOn();
m[0].setVelocityTick(SPEED);
m[1].setVelocityTick(-SPEED/2);
m[0].startMotion();
m[1].startMotion();
sleep(FORW_TIME);
m[0].stopMotion();
m[1].stopMotion();
m[0].turnOff();
m[1].turnOff();

sleep(5);

m[0].turnOn();
m[1].turnOn();
m[0].setVelocityTick(SPEED);
m[1].setVelocityTick(SPEED);
m[0].startMotion();
m[1].startMotion();
sleep(TURN_TIME);
m[0].stopMotion();
m[1].stopMotion();
m[0].turnOff();
m[1].turnOff();

sleep(5);

m[0].turnOn();
m[1].turnOn();
m[0].setVelocityTick(-SPEED);
m[1].setVelocityTick(-SPEED);
m[0].startMotion();
m[1].startMotion();
sleep(TURN_TIME);
m[0].stopMotion();
m[1].stopMotion();
m[0].turnOff();
m[1].turnOff();

sleep(5);

m[0].turnOn();
m[1].turnOn();
m[0].setVelocityTick(-SPEED);
m[1].setVelocityTick(SPEED);
m[0].startMotion();
m[1].startMotion();
sleep(FORW_TIME);
m[0].stopMotion();
m[1].stopMotion();
m[0].turnOff();
m[1].turnOff();

}
