/*******************************************************
 * @file driver.cpp
 * @author khshen (khshen@aceinna.com)
 * @brief 
 * @date 2021-05-31
 * 
 * @copyright Copyright (c) 2021
 * 
*******************************************************/
#include "driver.h"
const float r2d = 180/3.14159265;

#define     UART_BAUDRATE       (460800)                /**You should change it according to the actual setting**/
#define     UART_PORT_ID        ("/dev/ttyUSB0")        /**You should change it according to the actual setting**/
#define     UART_PARITY         0                       /**0:None; 1:Odd; 2:Even; 3:Mark; 4:Space**/
#define     UART_BYTESIZE       8
#define     UART_STOPBIT        (1)                     /**1:One Bit; 2:Two Bit; 1.5:One Point Five**/

RTKDriver::RTKDriver(ros::NodeHandle nh)
    : m_nh(nh)
{
    cout << "RTKDriver:RTKDriver()" << endl;
    m_pserial = nullptr;

    /* You need to change PortID and Baudrate, others use default*/
    m_rtk.m_port = string(UART_PORT_ID);
    m_rtk.m_baud = UART_BAUDRATE;

    ROS_INFO("device: %s", m_rtk.m_port.c_str());
    ROS_INFO("baud: %d", m_rtk.m_baud);

    rtk_pub_imu  = m_nh.advertise<rtk_msg::rtkmsg_imu>("topic_rtk_imu", 100);
    rtk_pub_gnss = m_nh.advertise<rtk_msg::rtkmsg_gnss>("topic_rtk_gnss", 100);
    rtk_pub_ins  = m_nh.advertise<rtk_msg::rtkmsg_ins>("topic_rtk_ins", 100);
}

RTKDriver::~RTKDriver()
{
    cout << "RTKDriver:~RTKDriver()" << endl;
}

void RTKDriver::Start()
{
    // Set and open serial port.
    m_pserial = new serial::Serial(m_rtk.m_port, m_rtk.m_baud, serial::Timeout::simpleTimeout(10));

    while (!m_pserial->isOpen())
    {
        ROS_WARN("Keep trying to open the device in 0.1 second period...");
        usleep(100000); // 100 ms
        m_pserial->open();
    }
    ROS_INFO("Device is opened successfully. [%s:%d]", m_rtk.m_port.c_str(), m_rtk.m_baud);
        
    m_mx_exit.lock();
    m_bexit = false;
    m_mx_exit.unlock();
    m_get_data_thread = std::thread(&RTKDriver::ThreadGetData, this);

    signal(SIGINT, SigintHandler);
    Spin();
}

void RTKDriver::Stop()
{
    cout << "RTKDriver::Stop()" << endl;
    if(m_pserial)
    {
        if(m_pserial->isOpen())
        {
            m_pserial->close();
        }
        SAFEDELETE(m_pserial);
    }

    m_mx_exit.lock();
    m_bexit = true;
    m_mx_exit.unlock();
    usleep(1000);
}

void RTKDriver::SigintHandler(int sig)
{
    // Capture Ctrl+C.
    ROS_INFO("shutting down!");
	ros::shutdown();
}

bool RTKDriver::Spin()
{
    double rate_ = 200; // Hz
    ros::Rate loop_rate(rate_);

    size_t max_sz = 200;
    uint8_t *buf = new uint8_t[max_sz];
    while (ros::ok())
    {
        /*
         * ros::ok() return false once:
         *  1. ros::shutdown() has been called and is finished
         *  2. SIGINT (Ctrl-C)
         *  3. ros::NodeHandles were destroyed.
         *  4. Node with same name appears in ROS network.
	     */

        size_t sz = m_pserial->read(buf, max_sz);

        m_mt_buf.lock();
        for (size_t i = 0; i < sz; i++)
        {
            m_buf.push(buf[i]);
        }
        m_mt_buf.unlock();

        ros::spinOnce();
        loop_rate.sleep();
    }

    Stop();

    return true;
};

void RTKDriver::ThreadGetData()
{
    cout << "RTKDriver::ThreadGetData()" << endl;

    const uint8_t HEADER[] = {0X55, 0X55};
    const uint8_t PACKAGE_TYPE_IDX = 2;
    const uint8_t PAYLOAD_LEN_IDX = 4;
    const int MAX_FRAME_LIMIT = 256;  // assume max len of frame is smaller than MAX_FRAME_LIMIT.

    deque<uint8_t> sync_pattern (2, 0);
	uint8_t* buf = new uint8_t[MAX_FRAME_LIMIT];
    bool find_header = false;
    uint16_t payload_len = 0;
    uint8_t d = 0;
	int idx = 0;

    while (1)
    {
        m_mx_exit.lock();
        if(m_bexit) 
        {
            m_mx_exit.unlock();
            break;
        }
        m_mx_exit.unlock();

        m_mt_buf.lock();
        if (m_buf.empty()) 
        {
            m_mt_buf.unlock();
            usleep(1000); // 1ms
            continue;
        }
        d = m_buf.front();
        m_buf.pop();
        m_mt_buf.unlock();
        
        if(find_header)
        {
            buf[++idx] = d;

            if (PAYLOAD_LEN_IDX == idx)
            {
                payload_len = buf[PAYLOAD_LEN_IDX];
            }
            else if ( (2 + 2 + 1 + payload_len + 2) == (idx + 1))  // 2: len of header 'UU'; 2: package type 'a1'; 1: payload len; 2:len of checksum.
            {    
                find_header = false;
                // checksum
                uint16_t packet_crc = 256 * buf[idx-1] + buf[idx];
                if (packet_crc == calcCRC(&buf[PACKAGE_TYPE_IDX], idx-3)) // 4: len of header 'UU', and len of checksum.
                {
                    // find a whole frame
                    ParseFrame(buf, idx+1);
                }
                else
                {
                    cout << "CRC error! : " << Bytestohexstring(buf, idx + 1) << endl;
                }
            }

            if (payload_len > MAX_FRAME_LIMIT || idx > MAX_FRAME_LIMIT)
            {
                find_header = false;
                payload_len = 0;
    			memset(buf, MAX_FRAME_LIMIT, 0);
            }
        }
        else  // if hasn't found header 'UU'
        {
            sync_pattern.emplace_front(d);
            if (sync_pattern[0] == HEADER[0] && sync_pattern[1] == HEADER[1])
            {
                idx = 1;
    			memcpy(buf, HEADER, 2);
                find_header = true;
                sync_pattern[0] = 0;
            }
            sync_pattern.resize(2, 0);
        }
    }
}

void RTKDriver::ParseFrame(uint8_t* frame, uint16_t len)
{
    const uint8_t PACKAGE_TYPE_IDX = 2;

    string packetType;
    packetType.push_back(frame[PACKAGE_TYPE_IDX]);
    packetType.push_back(frame[PACKAGE_TYPE_IDX + 1]);

    if(m_rtk.m_packetType.compare(packetType) != 0)
    {
        m_rtk.m_packetType = packetType;
        ROS_INFO("PACKET TYPE:%s", packetType.c_str());
    }

    if (m_rtk.m_packetType.compare("s1") == 0)
    {
        Handle_RtkIMUMessage(frame, len);
    }
    else if (m_rtk.m_packetType.compare("g1") == 0)
    {
        Handle_RtkGNSSMessage(frame, len);
    }
    else if (m_rtk.m_packetType.compare("i1") == 0)
    {
        Handle_RtkINSMessage(frame, len);
    }
    else
    {
        ROS_WARN("Unknown Packet Type!");
    }
}

/*******************************************************************************
* FUNCTION: calcCRC calculates a 2-byte CRC on serial data using 
*           CRC-CCITT 16-bit standard maintained by the ITU 
*           (International Telecommunications Union). 
* ARGUMENTS: ptr is pointer to queue holding area to be CRCed
*            num is offset into buffer where to stop CRC calculation
* RETURNS: 2-byte CRC
*******************************************************************************/
uint16_t RTKDriver::calcCRC(uint8_t *ptr, uint32_t num)
{
    uint16_t crc = 0x1D0F; //non-augmented initial value equivalent to augmented initial value 0xFFFF
    for (uint32_t i = 0; i < num; i++)
    {
        crc ^= ptr[i] << 8;
        for(uint32_t j = 0; j < 8; j++) 
        {
            if(crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        } 
    }
    return crc;
}

string RTKDriver::Bytestohexstring(uint8_t* bytes,int bytelength)  
{  
  string str("");  
  string str2("0123456789ABCDEF");   
  for (int i = 0; i < bytelength; i++)
  {  
    int b = 0x0f & (bytes[i] >> 4);  
    str.append(1, str2.at(b));            
    b = 0x0f & bytes[i];  
    str.append(1, str2.at(b));  
    str += " ";
  }
  return str;  
}  

/***Handle RTK IMU Message***/
void RTKDriver::Handle_RtkIMUMessage(uint8_t* frame, uint16_t len)
{
    // Check size of frame.
    if (len - 7 != sizeof(stRTKIMUS1)) // 7: UU (2), packet type(2), playload length(1), crc(2).
    {
        ROS_WARN("RTK_IMU Frame size error!");
        return;
    }

    stRTKIMUS1 *S1Msg = (stRTKIMUS1*)(&frame[5]);
    rtk_msg::rtkmsg_imu imu_data;

    imu_data.header.frame_id = "RTK_IMU";
    imu_data.header.stamp = ros::Time::now();

    /*****to avoid byte alignment problem, 
     * it's better to Assignment one by one*****/
    imu_data.gps_week       = S1Msg->gps_week;
    imu_data.gps_millisecs  = S1Msg->gps_millisecs;
    imu_data.x_acceleration = S1Msg->x_acceleration;
    imu_data.y_acceleration = S1Msg->y_acceleration;
    imu_data.z_acceleration = S1Msg->z_acceleration;
    imu_data.x_gyro_rate    = S1Msg->x_gyro_rate;
    imu_data.y_gyro_rate    = S1Msg->y_gyro_rate;
    imu_data.z_gyro_rate    = S1Msg->z_gyro_rate;

    rtk_pub_imu.publish(imu_data);
}

/***Handle RTK INS Message***/
void RTKDriver::Handle_RtkINSMessage(uint8_t* frame, uint16_t len)
{
    // Check size of frame.
    if (len - 7 != sizeof(stRTKINSI1)) // 7: UU (2), packet type(2), playload length(1), crc(2).
    {
        ROS_WARN("RTK INS Frame size error!");
        return;
    }

    stRTKINSI1 *I1Msg = (stRTKINSI1*)(&frame[5]);
    rtk_msg::rtkmsg_ins ins_data;

    ins_data.header.frame_id = "RTK_INS";
    ins_data.header.stamp = ros::Time::now();

    /*****to avoid byte alignment problem, 
     * it's better to Assignment one by one*****/
    ins_data.gps_week       = I1Msg->gps_week;
    ins_data.gps_millisecs  = I1Msg->gps_millisecs;
    ins_data.ins_status     = I1Msg->ins_status;
    ins_data.ins_position_type  = I1Msg->ins_position_type;
    ins_data.latitude           = I1Msg->latitude;
    ins_data.longitude          = I1Msg->longitude;
    ins_data.height             = I1Msg->height;
    ins_data.north_vel          = I1Msg->north_vel;
    ins_data.east_vel           = I1Msg->east_vel;
    ins_data.up_vel             = I1Msg->up_vel;
    ins_data.roll               = I1Msg->roll;
    ins_data.pitch              = I1Msg->pitch;
    ins_data.heading            = I1Msg->heading;
    ins_data.latitude_std_deviation     = I1Msg->latitude_std_deviation;
    ins_data.longitude_std_deviation    = I1Msg->longitude_std_deviation;
    ins_data.height_std_deviation       = I1Msg->height_std_deviation;
    ins_data.north_vel_std_deviation    = I1Msg->north_vel_std_deviation;
    ins_data.east_vel_std_deviation     = I1Msg->east_vel_std_deviation;
    ins_data.up_vel_std_deviation       = I1Msg->up_vel_std_deviation;
    ins_data.roll_std_deviation         = I1Msg->roll_std_deviation;
    ins_data.pitch_std_deviation        = I1Msg->pitch_std_deviation;
    ins_data.heading_std_deviation      = I1Msg->heading_std_deviation;

    rtk_pub_ins.publish(ins_data);
}

/***Handle RTK GNSS Message***/
void RTKDriver::Handle_RtkGNSSMessage(uint8_t* frame, uint16_t len)
{
    // Check size of frame.
    if (len - 7 != sizeof(stRTKGNSSG1)) // 7: UU (2), packet type(2), playload length(1), crc(2).
    {
        ROS_WARN("RTK GNSS Frame size error!");
        return;
    }

    stRTKGNSSG1 *G1Msg = (stRTKGNSSG1*)(&frame[5]);
    rtk_msg::rtkmsg_gnss gnss_data;

    gnss_data.header.frame_id = "RTK_GNSS";
    gnss_data.header.stamp = ros::Time::now();

    /*****to avoid byte alignment problem, 
     * it's better to Assignment one by one*****/
    gnss_data.gps_week           = G1Msg->gps_week;
    gnss_data.gps_millisecs      = G1Msg->gps_millisecs;
    gnss_data.position_type      = G1Msg->position_type;
    gnss_data.latitude           = G1Msg->latitude;
    gnss_data.longitude          = G1Msg->longitude;
    gnss_data.height             = G1Msg->height;
    gnss_data.latitude_std_deviation        = G1Msg->latitude_std_deviation;
    gnss_data.longitude_std_deviation       = G1Msg->longitude_std_deviation;
    gnss_data.height_std_deviation          = G1Msg->height_std_deviation;
    gnss_data.num_of_satellites             = G1Msg->num_of_satellites;
    gnss_data.num_satellite_in_solution     = G1Msg->num_satellite_in_solution;
    gnss_data.hdop                          = G1Msg->hdop;
    gnss_data.diffage                       = G1Msg->diffage;
    gnss_data.north_vel                     = G1Msg->north_vel;
    gnss_data.east_vel                      = G1Msg->east_vel;
    gnss_data.up_vel                        = G1Msg->up_vel;
    gnss_data.north_vel_std_deviation       = G1Msg->north_vel_std_deviation;
    gnss_data.east_vel_std_deviation        = G1Msg->east_vel_std_deviation;
    gnss_data.up_vel_std_deviation          = G1Msg->up_vel_std_deviation;

    rtk_pub_gnss.publish(gnss_data);
}