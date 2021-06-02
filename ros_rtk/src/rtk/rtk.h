/*******************************************************
 * @file imu.h
 * @author SongYang (khshen@aceinna.com)
 * @brief 
 * @date 2021-05-01
 * 
 * @copyright Copyright (c) 2021
 * 
*******************************************************/

#pragma once
#include <iostream>
using namespace std;


class CRTK
{
public:
    string m_sn;
    string m_pn;
    string m_packetType;
    string m_arc;
    string m_port;
    int32_t m_baud;
    int m_odr;
public:
    CRTK();
    virtual ~CRTK();
};
