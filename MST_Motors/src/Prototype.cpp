#include "motorController.h"
#include <unistd.h>
#include <stdio.h>

int main()
{
printf("testing ttyUSB0\n");
motorController motor0("/dev/ttyUSB0");
motor0.turnOn();
motor0.setVelocityTick(2000);
motor0.startMotion();
sleep(5);
motor0.stopMotion();
motor0.turnOff();

sleep(5);

printf("testing ttyUSB1\n");
motorController motor1("/dev/ttyUSB1");
motor1.turnOn();
motor1.setVelocityTick(2000);
motor1.startMotion();
sleep(5);
motor1.stopMotion();
motor1.turnOff();

sleep(5);

printf("testing ttyUSB2\n");
motorController motor2("/dev/ttyUSB2");
motor2.turnOn();
motor2.setVelocityTick(2000);
motor2.startMotion();
sleep(5);
motor2.stopMotion();
motor2.turnOff();
}
