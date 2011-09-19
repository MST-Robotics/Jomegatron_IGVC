#include "JAUS_Controller.h"
#include <jaus/core/time.h>

JAUS_Controller::JAUS_Controller( ros::NodeHandle n )
{
    fault = false;
    initialize_subs_and_pubs(n);
    component.AccessControlService()->SetTimeoutPeriod(0);
    initialize_services();
    component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "JoeMegatron");
    
    if(component.Initialize(JAUS::Address(SUBSYSTEM_ID, NODE_ID, COMPONENT_ID)) == false)
    {
        ROS_ERROR("Failed to Initialize Component.");
        fault = true;
    }
    
    component.ManagementService()->SetStatus(JAUS::Management::Status::Standby);

    transportService->AddConnection(COP_IP, JAUS::Address(COP_SUBSYSTEM_ID, COP_NODE_ID, COP_COMPONENT_ID));
}

JAUS_Controller::~JAUS_Controller()
{
}

void JAUS_Controller::initialize_services()
{
    globalPoseSensor = new JAUS::GlobalPoseSensor();
    globalPoseSensor->SetSensorUpdateRate(25);      // Updates at 25 Hz (used for periodic events).
    component.AddService(globalPoseSensor);

    localPoseSensor = new JAUS::LocalPoseSensor();
    localPoseSensor->SetSensorUpdateRate(25);       // Updates at 25 Hz (used for periodic events).
    component.AddService(localPoseSensor);

    velocityStateSensor = new JAUS::VelocityStateSensor();
    velocityStateSensor->SetSensorUpdateRate(25);   // Updates at 25 Hz (used for periodic events).
    component.AddService(velocityStateSensor);

    component.AddService(new JAUS::ListManager());
    localWaypointListDriver = new JAUS::LocalWaypointListDriver();
    component.AddService(localWaypointListDriver);
    
    transportService = (JAUS::JUDP*)component.TransportService();
}

void JAUS_Controller::initialize_subs_and_pubs( ros::NodeHandle n )
{
    s_Midg = n.subscribe( "/midg", 1, &JAUS_Controller::MidgCallback, this );
    //~ s_Control = n.subscribe( /* TODO */, 1, ControlCallback );
    //~ s_Position = n.subscribe( /* TODO */, 1, PositionCallback );
    s_Motors = n.subscribe( "/cmd_vel", 1, &JAUS_Controller::MotorsCallback, this );
    
    //~ p_Control = n.advertise</* TODO */>(/* TODO */, 5 );
}

bool JAUS_Controller::run()
{
    bool status = true;
    if( component.ManagementService()->GetStatus() != JAUS::Management::Status::Shutdown )
    {   
        globalPoseSensor->SetGlobalPose(globalPose);
        localPoseSensor->SetLocalPose(globalPose);
        velocityStateSensor->SetVelocityState(velocityState);
    }
    else
    {
        status = false;
    }
    return status;
}

void JAUS_Controller::MidgCallback( const Midg::IMU::ConstPtr& msg )
{
    globalPose.SetLatitude( msg->latitude );
    globalPose.SetLongitude( msg->longitude );
    globalPose.SetAltitude( msg->altitude );
    globalPose.SetYaw( msg->heading );
    
    JAUS::Time t;
    t.SetCurrentTime();
    globalPose.SetTimeStamp(t);

    ROS_INFO("Midg data received");
}

//~ void JAUS_Controller::ControlCallback( MST_Control::/* TODO */::ConstPtr& msg )
//~ {
//~ }
//~ 
//~ void JAUS_Controller::PositionCallback( MST_Position::/* TODO */::ConstPtr& msg )
//~ {
//~ }

void JAUS_Controller::MotorsCallback( const geometry_msgs::Twist::ConstPtr& msg )
{
    velocityState.SetVelocityX( msg->linear.x );
    velocityState.SetYawRate( msg->angular.z );
    
    JAUS::Time t;
    t.SetCurrentTime();
    velocityState.SetTimeStamp( t );

    ROS_INFO("Motors data received");
}
