#include "JAUS_COP.h"
#include <jaus/core/time.h>

JAUS_COP::JAUS_COP( ros::NodeHandle n )
{
    fault = false;
    mst_jaus_discovered = false;
    mst_jaus_controlled = false;
    initialize_subs_and_pubs(n);
    component.AccessControlService()->SetTimeoutPeriod(0);
    initialize_services();
    component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "JoeMegatron");
    
    if(component.Initialize(JAUS::Address(COP_SUBSYSTEM_ID, COP_NODE_ID, COP_COMPONENT_ID)) == false)
    {
        ROS_ERROR("Failed to Initialize Component.");
        fault = true;
    }
    
    component.ManagementService()->SetStatus(JAUS::Management::Status::Standby);

    transportService->AddConnection(COP_IP, JAUS::Address(SUBSYSTEM_ID, NODE_ID, COMPONENT_ID));
}

JAUS_COP::~JAUS_COP()
{
}

void JAUS_COP::initialize_services()
{
    /*globalPoseSensor = new JAUS::GlobalPoseSensor();
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
    component.AddService(localWaypointListDriver);*/
    
    transportService = (JAUS::JUDP*)component.TransportService();
}

void JAUS_COP::initialize_subs_and_pubs( ros::NodeHandle n )
{
}

bool JAUS_COP::run()
{
    JAUS::Address source(COP_SUBSYSTEM_ID, COP_NODE_ID, COP_COMPONENT_ID);
    JAUS::Address destination(SUBSYSTEM_ID, NODE_ID, COMPONENT_ID);
    
    bool status = true;
    if( component.ManagementService()->GetStatus() != JAUS::Management::Status::Shutdown )
    {
        //Check for MST_JAUS until it is discovered
        if(!mst_jaus_discovered)
        {
            JAUS::Subsystem::Ptr subsystem = component.DiscoveryService()->GetSubsystem(SUBSYSTEM_ID);
            if(subsystem != NULL)
            {
                if(subsystem->GetComponent(destination))
                {
                    ROS_INFO("MST_JAUS component discovered.");
                    mst_jaus_discovered = true;
                }
            }
        }
        
        else if(!mst_jaus_controlled) //jaus_node was discovered, take control
        {
            //Take control of jaus_node component
            JAUS::RequestControl requestControl(destination, source);
            JAUS::ConfirmControl confirmControl(source, destination); //should be overwritten by response
            if(!component.Send(&requestControl, &confirmControl, 1000))
            {
                ROS_ERROR("RequestControl message failed.");
            }
            else
            {
                if(confirmControl.GetResponseCode() == 0)
                //0 == have control
                //2 == someone with higher authority has control
                //1 == misc reason component could not give control
                {
                    ROS_INFO("COP has control of jaus_node component.");
                    mst_jaus_controlled = true;
                }
            }
        }
        
        else //have control of jaus_node component
        {
            //Query/Report Status, should run every 5 seconds (but we'll run it more often for now)
            JAUS::QueryStatus queryStatus(destination, source);
            JAUS::ReportStatus reportStatus(source, destination);
            if(!component.Send(&queryStatus, &reportStatus, 1000))
            {
                ROS_ERROR("QueryStatus message failed.");
            }
            else
            {
                switch(reportStatus.GetStatus())
                {
                case JAUS::ReportStatus::Init:
                    ROS_INFO("Status: Init");
                    break;
                case JAUS::ReportStatus::Ready:
                    ROS_INFO("Status: Ready");
                    break;
                case JAUS::ReportStatus::Standby:
                    ROS_INFO("Status: Standby");
                    break;
                case JAUS::ReportStatus::Shutdown:
                    ROS_INFO("Status: Shutdown");
                    break;
                case JAUS::ReportStatus::Failure:
                    ROS_INFO("Status: Failure");
                    break;
                case JAUS::ReportStatus::Emergency:
                    ROS_INFO("Status: Emergency");
                    break;
                }
            }
            
            //TODO
        }
    }
    else
    {
        status = false;
    }
    return status;
}
