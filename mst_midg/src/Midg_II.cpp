/*******************************************************************************
 * File: Midg_II.cpp
 * Auth: Chris Bessent <cmbq76>
 *
 * Desc: Midg controller.  Parses data from Midg and posts to "Midg" topic.
 ******************************************************************************/

/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include "mst_midg/IMU.h"
#include "sensor_msgs/NavSatFix.h"

/***********************************************************
* Other includes
***********************************************************/
#include "Midg_II.h"
#include <list>
#include "Math.h"
#include "drivers/midgPacket.h"

/***********************************************************
* Global variables
***********************************************************/
double avg_mag_x = 0;
double avg_mag_y = 0;
double heading = 0;
double reading_count = 0;

// sensor_msgs::NavSatFix
const bool FIX_COVARIANCE = false;
const double FIXED_COVARIANCE_VALUE = 5000;

/***********************************************************
* Function prototypes
***********************************************************/
int  Open_MIDG_Connection();
void Process_MIDG_Packets( int );

/***********************************************************
* Message Callbacks
***********************************************************/
mst_midg::IMU           imu_msg;
sensor_msgs::NavSatFix  navSatFix_msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Midg");
    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<mst_midg::IMU>( "/midg", 1000 );
    ros::Publisher navSatFix_pub = n.advertise<sensor_msgs::NavSatFix>( "/fix", 1000 );
    navSatFix_msg.header.frame_id = "midg_link";

    /***********************************************************
    * Midg initialization
    ***********************************************************/
    int fd = Open_MIDG_Connection();

    while( ros::ok() )
    {
//        ROS_INFO("Midg alive!");
        Process_MIDG_Packets( fd );

        imu_pub.publish( imu_msg );
        navSatFix_pub.publish( navSatFix_msg );
    }

}

int Open_MIDG_Connection()
{
    //------------------------------------------------------------------
    // Create serial socket port
    //------------------------------------------------------------------
    int fd;
    struct termios newtio;

    fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK );
    if (fd <0) {perror("/dev/ttyUSB1"); exit(-1); }

    memset( &newtio, 0x00, sizeof(newtio) );

    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | IGNBRK;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0 & !ECHO;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 250; /* blocking read until 1 chars received */

    tcsetattr(fd,TCSANOW,&newtio);

    return fd;
} /* Start_MIDG_Connection() */

void Process_MIDG_Packets( int fd )
{
    static list<double>  GPSPV_LONG_DATA;
    static list<double>  GPSPV_LAT_DATA;

    msg_NAVPV       msg_navpv;
    msg_GPSPV       msg_gpspv;
    msg_NAVSENSE    msg_navsense;
    msg_UTCTIME     msg_utctime;
    msg_NAVHDG      msg_navhdg;
    msg_IMUMAG      msg_imumag;

    const unsigned int sample_size = 10;

    //ROS_INFO("getonepacket");
    midgPacket msg_temp = getonepacket( fd );
    switch( msg_temp.messageID )
    {
        case MIDG_MESSAGE_NAVPVDATA:
            //ROS_INFO("midg_message_navpvdata");
            msg_navpv = msg_temp.handle_msg_NAVPV();

            if( !msg_navpv.positionvalid )
            {
                imu_msg.position_valid = false;
                for(int i=0; i<9; ++i)
                    navSatFix_msg.position_covariance[i] = 0;
            }
            else
            {
                //ROS_INFO("AAAAAAA");
				if( GPSPV_LONG_DATA.size() >= sample_size )
                {
                    GPSPV_LONG_DATA.pop_front();
                }
                GPSPV_LONG_DATA.push_back( msg_navpv.xPos );
                imu_msg.longitude = average( GPSPV_LONG_DATA );
                navSatFix_msg.longitude = imu_msg.longitude;

                if( GPSPV_LAT_DATA.size() >= sample_size )
                {
                    GPSPV_LAT_DATA.pop_front();
                }
                GPSPV_LAT_DATA.push_back( msg_navpv.yPos );
                
                //imu_msg.heading = atan2( GPSPV_LAT_DATA.back() - GPSPV_LAT_DATA.front(), GPSPV_LONG_DATA.back() - GPSPV_LONG_DATA.front() );

                imu_msg.longitude = msg_navpv.xPos;
                imu_msg.latitude  = msg_navpv.yPos;
                imu_msg.altitude  = msg_navpv.zPos;
                imu_msg.position_valid = true;
                
                navSatFix_msg.longitude = msg_navpv.xPos;
                navSatFix_msg.latitude  = msg_navpv.yPos;
                navSatFix_msg.altitude  = msg_navpv.zPos;
                navSatFix_msg.position_covariance[0] = FIXED_COVARIANCE_VALUE;
                navSatFix_msg.position_covariance[4] = FIXED_COVARIANCE_VALUE;
                navSatFix_msg.position_covariance[8] = FIXED_COVARIANCE_VALUE;
            }
            break;

        case MIDG_MESSAGE_GPSPVDATA:
            //ROS_INFO("midg_message_gpspvdata");
            msg_gpspv = msg_temp.handle_msg_GPSPV();

            if( !msg_gpspv.gpsfixvalid )
            {
                imu_msg.position_valid = false;
                for(int i=0; i<9; ++i)
                    navSatFix_msg.position_covariance[i] = 0;
                imu_msg.heading_valid = false;
                imu_msg.position_accuracy = msg_gpspv.positionaccuracy;
            }
            else
            {
                //----------------------------------------------------
                // Average GPS position over N samples
                //----------------------------------------------------
                if( GPSPV_LONG_DATA.size() >= sample_size )
                {
                    GPSPV_LONG_DATA.pop_front();
                }
                GPSPV_LONG_DATA.push_back( msg_gpspv.GPS_PosX );
                imu_msg.longitude = average( GPSPV_LONG_DATA );
                navSatFix_msg.longitude = imu_msg.longitude;

                if( GPSPV_LAT_DATA.size() >= sample_size )
                {
                    GPSPV_LAT_DATA.pop_front();
                }
                GPSPV_LAT_DATA.push_back( msg_gpspv.GPS_PosY );
                imu_msg.latitude = average( GPSPV_LAT_DATA );
                navSatFix_msg.latitude = imu_msg.latitude;

		//imu_msg.heading = atan2( GPSPV_LAT_DATA.back() - GPSPV_LAT_DATA.front(), GPSPV_LONG_DATA.back() - GPSPV_LONG_DATA.front() );

                imu_msg.longitude = msg_gpspv.GPS_PosX;
                imu_msg.latitude  = msg_gpspv.GPS_PosY;
                imu_msg.altitude  = msg_gpspv.GPS_PosZ;
                imu_msg.position_valid = true;
                imu_msg.position_accuracy = msg_gpspv.positionaccuracy;
                
                navSatFix_msg.longitude = msg_gpspv.GPS_PosX;
                navSatFix_msg.latitude  = msg_gpspv.GPS_PosY;
                navSatFix_msg.altitude  = msg_gpspv.GPS_PosZ;
                float temp[] = {msg_gpspv.positionaccuracy, 0, 0,
                                0, msg_gpspv.positionaccuracy, 0,
                                0, 0, msg_gpspv.positionaccuracy};
                if(FIX_COVARIANCE)
                    temp[0] = FIXED_COVARIANCE_VALUE;
                    temp[4] = FIXED_COVARIANCE_VALUE;
                    temp[8] = FIXED_COVARIANCE_VALUE;
                for(int i=0; i<9; ++i)
                    navSatFix_msg.position_covariance[i] = temp[i];
            }
            break;

        case MIDG_MESSAGE_NAVSENSEDATA:
            //ROS_INFO("midg_message_navsensedata");
            msg_navsense = msg_temp.handle_msg_NAVSENSE();

            //~ //------------------------------------------------------
            //~ // Modify yaw for robot system
            //~ //
            //~ // IN: North = 0, East = +M_PI_2, West = -M_PI_2,
            //~ //     South = +-M_PI
            //~ //
            //~ // OUT: East = 0, North = +M_PI_2, West = +M_PI,
            //~ //      South = +3*M_PI_2
            //~ //------------------------------------------------------
            //~ msg_navsense.yaw *= -1;
            //~ msg_navsense.yaw += M_PI_2;
            //~ while( msg_navsense.yaw < 0 )
            //~ {
                //~ msg_navsense.yaw += 2*M_PI;
            //~ }
            //~ while( msg_navsense.yaw > 2*M_PI )
            //~ {
                //~ msg_navsense.yaw -= 2*M_PI;
            //~ }

            //------------------------------------------------------
            // Modify angular rate for robot system
            //------------------------------------------------------
            msg_navsense.zAngRate *= M_PI/180.;

            //~ imu_msg.heading        = msg_navsense.yaw;
            imu_msg.angular_rate       = msg_navsense.zAngRate;
            break;

        case MIDG_MESSAGE_UTCTIME:
//            ROS_INFO("midg_message_utctime");
            msg_utctime = msg_temp.handle_msg_UTCTIME();

            imu_msg.gps_time = ( msg_utctime.timestamp - 15 );
            navSatFix_msg.header.stamp = ros::Time( imu_msg.gps_time );
            break;

        case MIDG_MESSAGE_NAVHDGDATA:
            //ROS_INFO("midg_message_navhdgdata");
            msg_navhdg = msg_temp.handle_msg_NAVHDG();
            
            //~ msg_navhdg.magHeading *= (M_PI/180);
            //~ //------------------------------------------------------
            //~ // Modify yaw for robot system
            //~ //
            //~ // IN: North = 0, East = +M_PI_2, West = -M_PI_2,
            //~ //     South = +-M_PI
            //~ //
            //~ // OUT: East = 0, North = +M_PI_2, West = +M_PI,
            //~ //      South = +3*M_PI_2
            //~ //------------------------------------------------------
            //~ msg_navhdg.magHeading *= -1;
            //~ msg_navhdg.magHeading += M_PI_2;
            //~ while( msg_navhdg.magHeading < 0 )
            //~ {
                //~ msg_navhdg.magHeading += 2*M_PI;
            //~ }
            //~ while( msg_navhdg.magHeading > 2*M_PI )
            //~ {
                //~ msg_navhdg.magHeading -= 2*M_PI;
            //~ }
//~ 
            //~ imu_msg.heading = msg_navhdg.magHeading;
            imu_msg.speed = msg_navhdg.sog;
            break;

	case MIDG_MESSAGE_IMUMAG:
		msg_imumag = msg_temp.handle_msg_IMUMAG();
		if(reading_count <10)
		{
			avg_mag_y += msg_imumag.mag_y_sine*0.1;
			avg_mag_x += msg_imumag.mag_x_sine*0.1;
			reading_count++;
		}
		else
		{
			reading_count = 0;
			heading = atan2(avg_mag_y,avg_mag_x);
			heading *= -1;
			heading += M_PI_2;
			if(heading < 0 )
			{
				heading += 2*M_PI;
			}
			while( heading > 2*M_PI )
			{			
				heading -= 2*M_PI;
			}
			//imu_msg.heading = avg_mag_x;
			//imu_msg.gps_time = avg_mag_y;
			avg_mag_x = 0;
			avg_mag_y = 0;
			imu_msg.heading = heading;
		}	
		
		break;
    default:
            //ROS_INFO("default message");
        break;
    } /* switch( msg_temp.messageID ) */
}  /* Process_MIDG_Packets() */
