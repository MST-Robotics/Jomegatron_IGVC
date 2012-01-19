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
    
    controlCallback = new ControlCallback(this);
    component.TransportService()->RegisterCallback(JAUS::REQUEST_CONTROL, controlCallback);
    component.TransportService()->RegisterCallback(JAUS::RESUME, controlCallback);
    component.TransportService()->RegisterCallback(JAUS::STANDBY, controlCallback);
    component.TransportService()->RegisterCallback(JAUS::SHUTDOWN, controlCallback);
    
    waypointCallback = new WaypointCallback(this);
    component.TransportService()->RegisterCallback(JAUS::SET_LOCAL_POSE, waypointCallback);
    component.TransportService()->RegisterCallback(JAUS::SET_ELEMENT, waypointCallback);
    component.TransportService()->RegisterCallback(JAUS::EXECUTE_LIST, waypointCallback);
    
    queryCallback = new QueryCallback(this);
    component.TransportService()->RegisterCallback(JAUS::QUERY_ELEMENT_LIST, queryCallback);
    component.TransportService()->RegisterCallback(JAUS::QUERY_ELEMENT_COUNT, queryCallback);
    component.TransportService()->RegisterCallback(JAUS::QUERY_ACTIVE_ELEMENT, queryCallback);
    //component.TransportService()->RegisterCallback(JAUS::QUERY_TRAVEL_SPEED, queryCallback);
    component.TransportService()->RegisterCallback(JAUS::QUERY_LOCAL_WAYPOINT, queryCallback);
    
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
    s_State = n.subscribe( "/jaus_in", 1, &JAUS_Controller::StateCallback, this );
    
    p_Control = n.advertise<MST_JAUS::JAUS_out>( "/jaus_out", 5 );
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

void JAUS_Controller::StateCallback(const MST_JAUS::JAUS_in::ConstPtr& msg)
{
    //Global Pose
    if(msg->position_valid)
    {
        globalPose.SetLatitude(msg->latitude);
        globalPose.SetLongitude(msg->longitude);
        globalPose.SetAltitude(msg->altitude);
        JAUS::Time t;
        t.SetCurrentTime(); //TODO convert msg->gps_time to JAUS::Time
        globalPose.SetTimeStamp(t);
    }
    if(msg->heading_valid)
        globalPose.SetYaw(msg->heading);
    
    //Velocty State
    if(msg->speed_valid)
        velocityState.SetVelocityX(msg->speed);
    if(msg->heading_valid)
        velocityState.SetYawRate(msg->angular_rate);
    if(msg->speed_valid || msg->heading_valid)
    {
        JAUS::Time t;
        t.SetCurrentTime();
        velocityState.SetTimeStamp(t);
    }
    
    //Waypoint List
    if(msg->waypoint_list_valid) {
        m_activeWaypoint = msg->active_waypoint_id;
        m_waypointID = msg->waypoint_id;
        m_waypointPreviousID = msg->waypoint_previous_id;
        m_waypointNextID = msg->waypoint_next_id;
        m_waypointX = msg->waypoint_x;
        m_waypointY = msg->waypoint_y;
    }

    ROS_INFO("State data received");
}

void JAUS_Controller::ControlCallback::ProcessMessage(const JAUS::Message* message)
{
    MST_JAUS::JAUS_out msg;
    
    if(message->GetMessageCode() == JAUS::REQUEST_CONTROL) {
        ROS_INFO("RequestControl JAUS message received.");
        msg.request_control = true;
    }
    else if(message->GetMessageCode() == JAUS::RESUME) {
        ROS_INFO("Resume JAUS message received.");
        parent->component.ManagementService()->SetStatus(JAUS::Management::Status::Ready);
        msg.request_resume = true;
    }
    else if(message->GetMessageCode() == JAUS::STANDBY) {
        ROS_INFO("Standby JAUS message received.");
        parent->component.ManagementService()->SetStatus(JAUS::Management::Status::Standby);
        msg.request_standby = true;
    }
    else if(message->GetMessageCode() == JAUS::SHUTDOWN) {
        ROS_INFO("Shutdown JAUS message received.");
        parent->component.ManagementService()->SetStatus(JAUS::Management::Status::Shutdown);
        msg.request_shutdown = true;
    }
    
    parent->p_Control.publish(msg);
}

void JAUS_Controller::WaypointCallback::ProcessMessage(const JAUS::Message* message)
{
    MST_JAUS::JAUS_out msg;
    
    if(message->GetMessageCode() == JAUS::SET_LOCAL_POSE) {
        ROS_INFO("SetLocalPose JAUS message received.");
        msg.set_local_pose = true;
        const JAUS::SetLocalPose* setLocalPose = dynamic_cast<const JAUS::SetLocalPose*>(message);
        msg.waypoint_pose_x.push_back(setLocalPose->GetX());
        msg.waypoint_pose_y.push_back(setLocalPose->GetY());
        msg.pose_yaw.push_back(setLocalPose->GetYaw());
    }
    else if(message->GetMessageCode() == JAUS::SET_ELEMENT) {
        ROS_INFO("SetElement JAUS message received.");
        msg.set_waypoints = true;
        const JAUS::SetElement* setElement = dynamic_cast<const JAUS::SetElement*>(message);
        for(unsigned int i = 0; i < setElement->GetElementList()->size(); ++i) {
            const JAUS::Element* element = &setElement->GetElementList()->at(i);
            JAUS::SetLocalWaypoint waypoint;
            waypoint.Read(element->mPayload);
            msg.waypoint_id.push_back(element->mID);
            msg.waypoint_previous_id.push_back(element->mPrevID);
            msg.waypoint_next_id.push_back(element->mNextID);
            msg.waypoint_pose_x.push_back(waypoint.GetX());
            msg.waypoint_pose_y.push_back(waypoint.GetY());
            std::cout<<"id: "<<element->mID<<' '<<waypoint.GetX()<<' '<<waypoint.GetY()<<std::endl;
        }
    }
    else if(message->GetMessageCode() == JAUS::EXECUTE_LIST) {
        ROS_INFO("ExecuteList JAUS message received.");
        msg.execute_waypoints = true;
        const JAUS::ExecuteList* executeList = dynamic_cast<const JAUS::ExecuteList*>(message);
        msg.speed = executeList->GetSpeed();
    }
    
    parent->p_Control.publish(msg);
}

void JAUS_Controller::QueryCallback::ProcessMessage(const JAUS::Message* message)
{
    JAUS::Address source = JAUS::Address(SUBSYSTEM_ID, NODE_ID, COMPONENT_ID);
    JAUS::Address destination = JAUS::Address(COP_SUBSYSTEM_ID, COP_NODE_ID, COP_COMPONENT_ID);
    
    if(message->GetMessageCode() == JAUS::QUERY_ELEMENT_LIST) {
        ROS_INFO("QueryElementList JAUS message received.");
        JAUS::ReportElementList reportElementList(destination, source);
        for(uint16_t i=0; i < parent->m_waypointID.size(); ++i) {
            reportElementList.GetElementList()->push_back(parent->m_waypointID.at(i));
        }
        if(!parent->component.Send(&reportElementList))
            ROS_ERROR("ReportElementList message failed.");
    }
    else if(message->GetMessageCode() == JAUS::QUERY_ELEMENT_COUNT) {
        ROS_INFO("QueryElementCount JAUS message received.");
        JAUS::ReportElementCount reportElementCount(destination, source);
        reportElementCount.SetElementCount(parent->m_waypointID.size());
        if(!parent->component.Send(&reportElementCount))
            ROS_ERROR("ReportElementCount message failed.");
    }
    else if(message->GetMessageCode() == JAUS::QUERY_ACTIVE_ELEMENT) {
        ROS_INFO("QueryActiveElement JAUS message received.");
        JAUS::ReportActiveElement reportActiveElement(destination, source);
        reportActiveElement.SetElementUID(parent->m_activeWaypoint);
        if(!parent->component.Send(&reportActiveElement))
            ROS_ERROR("ReportActiveElement message failed.");
    }
    /*else if(message->GetMessageCode() == JAUS::QUERY_TRAVEL_SPEED) {
        ROS_INFO("QueryTravelSpeed JAUS message received.");
        //TODO send latest from MST_Control
    }*/
    else if(message->GetMessageCode() == JAUS::QUERY_LOCAL_WAYPOINT) {
        ROS_INFO("QueryLocalWayoint JAUS message received.");
        JAUS::ReportLocalWaypoint reportLocalWaypoint(destination, source);
        for(uint16_t i=0; i < parent->m_waypointID.size(); ++i) {
            if(parent->m_waypointID.at(i) == parent->m_activeWaypoint) {
                reportLocalWaypoint.SetX(parent->m_waypointX.at(i));
                reportLocalWaypoint.SetY(parent->m_waypointY.at(i));
                break;
            }
        }
        if(!parent->component.Send(&reportLocalWaypoint))
            ROS_ERROR("ReportLocalWaypoint message failed.");
    }
}
