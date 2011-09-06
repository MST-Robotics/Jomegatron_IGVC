#include "motorController.h"

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

#include <ros/ros.h>

motorController::motorController(char* device)
{
    char buffer[20] = "";

    m_status = MOTOR_OFF;
    m_velocity = 0;
    m_fd = -1;
    strcpy( m_name, "" );    
    
    if( !device )
    {
        m_status = MOTOR_ERROR;
        return;
    }
    else
    {
        strcpy( m_name, device );
        if( ( m_fd = open(m_name,  O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0 )
        {
            ROS_ERROR("Port %s: failed to open", m_name);
            m_status = MOTOR_ERROR;
            return;
        }
        
	//Set the baud rate on the new port to 19200
	struct termios tty;
	bzero(&tty, sizeof(tty));
	tty.c_cflag = B19200 | CS8 | CLOCAL | CREAD;
	tty.c_iflag = ICRNL;
	tty.c_oflag = 0;
	tty.c_lflag = ICANON;
	if( tcflush(m_fd, TCIFLUSH) < 0 )
	{
		ROS_ERROR("Error flushing old settings");
		return;
	}
	if( tcsetattr(m_fd, TCSANOW, &tty) )
	{
		ROS_ERROR("Error applying new serial port settings");
		return;
	}
	ROS_INFO("Serial port %s ready with file descriptor %d", m_name, m_fd);
	
        strcpy( buffer, "\rUM=2\r\0" );
        if( write( m_fd, buffer, strlen(buffer) ) < 0 )
        {
            ROS_ERROR("Port %s: failed to write UM", m_name);
            m_status = MOTOR_ERROR;
            return;
        }
    }
}

bool motorController::setVelocityTick(int velocity)
{
    char buffer[20];
    if( (m_status == MOTOR_ON)
     || (m_status == MOTOR_MOVING) )
    {
        m_velocity = velocity;
        sprintf( buffer, "\rJV=%14d\r", m_velocity );
        if( write( m_fd, buffer, strlen(buffer) ) < 0 )
        {
            ROS_ERROR("Port %s: failed to write JV", m_name);
            m_status = MOTOR_ERROR;
            return false;
        }
    }
    else
    {
        ROS_WARN("Trying to set velocity on disabled motor");
        return false;
    }
    return true;
}

bool motorController::turnOn()
{
    char buffer[20];
    strcpy( buffer, "\rmo=1\r" );
    if( write( m_fd, buffer, strlen(buffer)) < 0 )
    {
        ROS_ERROR("Port %s: failed to write MO=1", m_name);
        m_status = MOTOR_ERROR;
        return false;
    }
    m_status = MOTOR_ON;
    return true;
}

bool motorController::turnOff()
{
    char buffer[20];
    strcpy( buffer, "\rmo=0\r" );
    if( write( m_fd, buffer, strlen(buffer)) < 0 )
    {
        ROS_ERROR("Port %s: failed to write MO=0", m_name);
        m_status = MOTOR_ERROR;
        return false;
    }
    m_status = MOTOR_OFF;
    return true;
}

bool motorController::startMotion()
{
    char buffer[20];
    strcpy( buffer, (char*)"\rBG\r" );
    if( write( m_fd, buffer, strlen(buffer)) < 0 )
    {
        ROS_ERROR("Port %s: failed to write BG", m_name);
        m_status = MOTOR_ERROR;
        return false;
    }
    m_status = MOTOR_MOVING;
    return true;
}

bool motorController::stopMotion()
{
    char buffer[20];
    strcpy( buffer, (char*)"\rST\r" );
    if( write( m_fd, buffer, strlen(buffer)) < 0 )
    {
        ROS_ERROR("Port %s: failed to write ST", m_name);
        m_status = MOTOR_ERROR;
        return false;
    }
    m_status = MOTOR_ON;
    return true;
}
