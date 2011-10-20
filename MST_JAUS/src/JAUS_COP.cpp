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
            printStatus(reportStatus.GetStatus());
        }
        
        return reportStatus.GetStatus();
    }
    
    return JAUS::ReportStatus::Shutdown;
}

void JAUS_COP::printStatus(JAUS::Byte status) const
{
    switch(status)
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

void* JAUS_COP::getInput(void* ptr)
{
    ROS_INFO("Input thread started.");
    
    JAUS_COP* cop = (JAUS_COP*)ptr;
    
    //current waypoint element ID;
    JAUS::UShort elementID = 1;
    JAUS::UShort startElement = elementID;
    
    std::string input;
    bool stop = false; //need stop since will be waiting on cin rather than looping and checking mst_jaus_status
    while(!stop && cop->mst_jaus_status != JAUS::ReportStatus::Shutdown)
    {
        std::cout << "> ";
        std::cin.clear();
        fflush(stdin);
        std::cin >> input;
        //TODO break into args
        
        if(input == "help")
        {
            std::cout << "Supported Messages:\n"
                      << "  System Management:\n"
                      << "    queryStatus\n"
                      << "    resume\n"
                      << "    standby\n"
                      << "    shutdown\n"
                      << "  Velocity State Report:\n"
                      << "    queryVelocity\n"
                      << "  Position and Orientation Report:\n"
                      << "    setLocalPose\n"
                      << "    queryLocalPose\n"
                      << "  Waypoint Navigation:\n"
                      << "    setElement\n"
                      << "    queryElementList\n"
                      << "    queryElementCount\n"
                      << "    executeList\n"
                      << "    queryActiveElement\n"
                      << "    queryTravelSpeed\n"
                      << "    queryLocalWaypoint\n"
                      << std::endl;
        }
        
        // System Management
        else if(input == "queryStatus")
        {
            JAUS::QueryStatus queryStatus(cop->destination, cop->source);
            JAUS::ReportStatus reportStatus(cop->source, cop->destination);
            if(!cop->component.Send(&queryStatus, &reportStatus, 1000))
                ROS_ERROR("QueryStatus message failed.");
            else
                cop->printStatus(reportStatus.GetStatus());
        }
        else if(input == "resume")
        {
            JAUS::Resume resume(cop->destination, cop->source);
            if(!cop->component.Send(&resume))
                ROS_ERROR("Resume message failed.");
        }
        else if(input == "standby")
        {
            JAUS::Standby standby(cop->destination, cop->source);
            if(!cop->component.Send(&standby))
                ROS_ERROR("Standby message failed.");
        }
        else if(input == "shutdown")
        {
            JAUS::Shutdown shutdown(cop->destination, cop->source);
            if(!cop->component.Send(&shutdown))
                ROS_ERROR("Shutdown message failed.");
            else
                stop = true;
        }
        
        // Velocity State Report
        else if(input == "queryVelocity")
        {
            JAUS::QueryVelocityState queryVelocityState(cop->destination, cop->source);
            JAUS::ReportVelocityState reportVelocityState(cop->source, cop->destination);
            if(!cop->component.Send(&queryVelocityState, &reportVelocityState, 1000))
                ROS_ERROR("QueryVelocityState message failed.");
            else
            {
                std::stringstream ss;
                ss << "Velocity X: " << reportVelocityState.GetVelocityX() << ' ';
                ss << "Yaw Rate: " << reportVelocityState.GetYawRate() << ' ';
                ss << "Time Stamp: " << reportVelocityState.GetTimeStamp().ToUInt();
                ROS_INFO(ss.str().c_str());
            }
        }
        
        // Position and Orientation Report
        else if(input == "setLocalPose")
        {
            //get pose from user
            double x;
            double y;
            double yaw;
            std::cin.clear();
            fflush(stdin);
            std::cout << "Enter pose: X Y Yaw\n  > ";
            std::cin >> x;
            std::cin >> y;
            std::cin >> yaw;
            
            JAUS::SetLocalPose setLocalPose(cop->destination, cop->source);
            setLocalPose.SetX(x);
            setLocalPose.SetY(y);
            setLocalPose.SetYaw(yaw);
            if(!cop->component.Send(&setLocalPose))
                ROS_ERROR("SetLocalPose message failed.");
        }
        else if(input == "queryLocalPose")
        {
            JAUS::QueryLocalPose queryLocalPose(cop->destination, cop->source);
            JAUS::ReportLocalPose reportLocalPose(cop->source, cop->destination);
            if(!cop->component.Send(&queryLocalPose, &reportLocalPose, 1000))
                ROS_ERROR("QueryLocalPose message failed.");
            else
            {
                std::stringstream ss;
                ss << "X: " << reportLocalPose.GetX() << ' ';
                ss << "Y: " << reportLocalPose.GetY() << ' ';
                ss << "Time Stamp: " << reportLocalPose.GetTimeStamp().ToUInt();
                ROS_INFO(ss.str().c_str());
            }
        }
        
        // Waypoint Navigation
        else if(input == "setElement")
        {
            double x;
            double y;
            std::cin.clear();
            fflush(stdin);
            std::cout << "Enter coordinate: X Y\n  > ";
            std::cin >> x;
            std::cin >> y;
            
            JAUS::SetElement setElement(cop->destination, cop->source);
            JAUS::ConfirmElementRequest confirmElementRequest(cop->source, cop->destination);
            JAUS::SetLocalWaypoint* setLocalWaypoint = new JAUS::SetLocalWaypoint(cop->destination, cop->source);
            setLocalWaypoint->SetX(x);
            setLocalWaypoint->SetY(y);
            JAUS::Element element(elementID, 0, 0);
            setElement.SetRequestID(elementID);
            element.mpElement = setLocalWaypoint;
            setElement.GetElementList()->push_back(element);
            if(!cop->component.Send(&setElement, &confirmElementRequest, 1000))
                ROS_ERROR("SetElement failed to add element. ConfirmElementRequest not received.");
            
            //DO NOT ENABLE THE NEXT LINE, JAUS::Element::~Element does it already
            //delete setLocalWaypoint;
            std::cout << "Added element, UID: " << elementID << std::endl;
            std::cout << "FOR NOW ONLY ELEMENT UID 1 or " << startElement << " WILL EXECUTE" << std::endl;
            elementID++;
        }
        else if(input == "queryElementList")
        {
            JAUS::QueryElementList queryElementList(cop->destination, cop->source);
            JAUS::ReportElementList reportElementList(cop->source, cop->destination);
            if(!cop->component.Send(&queryElementList, &reportElementList, 1000))
                ROS_ERROR("QueryElementList message failed.");
            else
            {
                for(unsigned int i = 0; i < reportElementList.GetElementList()->size(); ++i) 
                {
                    std::stringstream ss;
                    ss << "Element UID: " << reportElementList.GetElementList()->at(i);
                    ROS_INFO(ss.str().c_str());
                }
            }
        }
        else if(input == "queryElementCount")
        {
            JAUS::QueryElementCount queryElementCount(cop->destination, cop->source);
            JAUS::ReportElementCount reportElementCount(cop->source, cop->destination);
            if(!cop->component.Send(&queryElementCount, &reportElementCount, 1000))
                ROS_ERROR("QueryElementCount message failed.");
            else
            {
                std::stringstream ss;
                ss << "Element Count: " << reportElementCount.GetElementCount();
                ROS_INFO(ss.str().c_str());
            }
        }
        else if(input == "executeList")
        {
            double speed;
            std::cin.clear();
            fflush(stdin);
            std::cout << "Enter speed:\n  > ";
            std::cin >> speed;
            
            JAUS::ExecuteList executeList(cop->destination, cop->source);
            executeList.SetElementUID(startElement);
            executeList.SetSpeed(speed);
            if(!cop->component.Send(&executeList))
                ROS_ERROR("ExecuteList message failed.");
                
            startElement = elementID; //since elements deleted when reached
        }
        else if(input == "queryActiveElement")
        {
            JAUS::QueryActiveElement queryActiveElement(cop->destination, cop->source);
            JAUS::ReportActiveElement reportActiveElement(cop->source, cop->destination);
            if(!cop->component.Send(&queryActiveElement, &reportActiveElement, 1000))
                ROS_ERROR("QueryActiveElement message failed.");
            else
            {
                std::stringstream ss;
                ss << "Element UID: " << reportActiveElement.GetElementUID();
                ROS_INFO(ss.str().c_str());
            }
        }
        else if(input == "queryTravelSpeed")
        {
            JAUS::QueryTravelSpeed queryTravelSpeed(cop->destination, cop->source);
            JAUS::ReportTravelSpeed reportTravelSpeed(cop->source, cop->destination);
            if(!cop->component.Send(&queryTravelSpeed, &reportTravelSpeed, 1000))
                ROS_ERROR("QueryTravelSpeed message failed.");
            else
            {
                std::stringstream ss;
                ss << "Speed: " << reportTravelSpeed.GetSpeed();
                ROS_INFO(ss.str().c_str());
            }
        }
        else if(input == "queryLocalWaypoint")
        {
            JAUS::QueryLocalWaypoint queryLocalWaypoint(cop->destination, cop->source);
            JAUS::ReportLocalWaypoint reportLocalWaypoint(cop->source, cop->destination);
            if(!cop->component.Send(&queryLocalWaypoint, &reportLocalWaypoint, 1000))
                ROS_ERROR("QueryLocalWaypoint message failed.");
            else
            {
                std::stringstream ss;
                ss << "X: " << reportLocalWaypoint.GetX() << ' ';
                ss << "Y: " << reportLocalWaypoint.GetY();
                ROS_INFO(ss.str().c_str());
            }
        }
        
        else
        {
            std::cout << "Unrecognized command, 'help' for command list." << std::endl;
        }
    }
    
    ROS_INFO("Input thread terminating.");
    return ptr;
}
