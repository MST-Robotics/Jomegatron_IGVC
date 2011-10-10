#include "JAUS_COP.h"
#include <jaus/core/time.h>

JAUS_COP::JAUS_COP( ros::NodeHandle n )
{
    fault = false;
    
    source = JAUS::Address(COP_SUBSYSTEM_ID, COP_NODE_ID, COP_COMPONENT_ID);
    destination = JAUS::Address(SUBSYSTEM_ID, NODE_ID, COMPONENT_ID);
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
    transportService = (JAUS::JUDP*)component.TransportService();
}

void JAUS_COP::initialize_subs_and_pubs( ros::NodeHandle n )
{
}

bool JAUS_COP::run()
{
    bool status = true;
    if( component.ManagementService()->GetStatus() != JAUS::Management::Status::Shutdown )
    {
        //Check for MST_JAUS until it is discovered
        if(!mst_jaus_discovered)
            mst_jaus_discovered = discover(&status);
        
        else if(!mst_jaus_controlled) //jaus_node was discovered, take control
        {
            mst_jaus_controlled = requestControl();
            pthread_create(&inputThread, NULL, getInput, this);
        }
        
        else //have control of jaus_node component
        {
            //Query/Report Status, run every 5 seconds
            if(std::time(NULL) >= statusTimer+5)
            {
                mst_jaus_status = requestStatus();
                statusTimer = std::time(NULL);
            }
            
            if(mst_jaus_status == JAUS::ReportStatus::Shutdown)
            {
                //thread should kill itself
                mst_jaus_discovered = false;
                mst_jaus_controlled = false;
            }
        }
    }
    else
    {
        status = false;
    }
    return status;
}

bool JAUS_COP::discover(bool* run)
{
    JAUS::Subsystem::Ptr subsystem = component.DiscoveryService()->GetSubsystem(SUBSYSTEM_ID);
    if(subsystem != NULL)
    {
        if(subsystem->GetComponent(destination))
        {
            ROS_INFO("MST_JAUS component discovered.");
            
            JAUS::QueryServices queryServices(destination, source);
            JAUS::ReportServices reportServices(source, destination);
            if(!component.Send(&queryServices, &reportServices, 1000))
            {
                ROS_ERROR("QueryServices message failed, MST_JAUS service was probably shutdown. COP exiting.");
                *run = false;
            }
            
            else
            {
                JAUS::ReportServices::Services::iterator is;
                for(is = reportServices.GetServices()->begin(); is != reportServices.GetServices()->end(); is++)
                {
                    JAUS::ReportServices::Record::List::iterator ir;
                    for(ir = is->second.begin(); ir != is->second.end(); ir++)
                    {
                        ROS_INFO("Found a service.");
                    }
                }
            }
            return true;
        }
    }
    
    return false;
}

bool JAUS_COP::requestControl()
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
            return true;
        }
    }
    
    return false;
}

JAUS::Byte JAUS_COP::requestStatus()
{
    JAUS::QueryStatus queryStatus(destination, source);
    JAUS::ReportStatus reportStatus(source, destination);
    if(!component.Send(&queryStatus, &reportStatus, 1000))
    {
        ROS_ERROR("QueryStatus message failed.");
    }
    else
    {
        if(PRINT_STATUS_MESSAGES)
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
        
        return reportStatus.GetStatus();
    }
    
    return JAUS::ReportStatus::Shutdown;
}

void* JAUS_COP::getInput(void* ptr)
{
    ROS_INFO("Input thread started.");
    
    JAUS_COP* cop = (JAUS_COP*)ptr;
    
    std::string input;
    bool stop = false; //need stop since will be waiting on cin rather than looping and checking mst_jaus_status
    while(!stop && cop->mst_jaus_status != JAUS::ReportStatus::Shutdown)
    {
        std::cout << "> ";
        std::cin.clear();
        fflush(stdin);
        std::cin >> input;
        
        if(input == "shutdown")
        {
            JAUS::Shutdown shutdown(cop->destination, cop->source);
            if(!cop->component.Send(&shutdown))
                ROS_ERROR("Shutdown message failed.");
            else
                stop = true;
        }
    }
    
    ROS_INFO("Input thread terminating.");
    return ptr;
}
