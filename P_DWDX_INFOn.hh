/*
P_DWDX_INFOn.hh

This C++ header file defines the NML Messages for P_DWDX_INFO
Template Version 1.1

MODIFICATIONS:
Wed Dec 06 16:46:23 CST 2017	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef P_DWDX_INFON_HH
#define P_DWDX_INFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define P_DWDX_INFO_MSG_TYPE 157000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

class P_DWDX_INFO_MSG : public NMLmsg
{
public:

	//Constructor
    P_DWDX_INFO_MSG();

//    P_DWDX_INFO_MSG():
//        NMLmsg(P_DWDX_INFO_MSG_TYPE,sizeof(P_DWDX_INFO_MSG)){};


	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    //// 除了——gps外，其余为融合后的定位定向结果；姿态角度倾向于用INS的测量姿态，而不是估计出来的姿态；
    ///时间戳
    double time_stamp;
    ///无人车全局坐标X值(北)(融合后的坐标x)，值域，0～99999999，坐标刻度为0.1m
    unsigned int global_x;
    ///无人车全局坐标Y值(东)(融合后的坐标y)，值域，0～99999999，坐标刻度为0.1m
    unsigned int global_y;
    ///无人车全局高程H值(地)(GPS only)，值域，0～99999999，坐标刻度为0.1m
    unsigned int global_h;
    ///区号(GPS only)，值域，0～60
    unsigned short int zone;
    ///经度坐标(融合后的经度)，值域，刻度0.000001度,范围-180~180度;
    int longitude;
    ///纬度坐标(融合后的纬度)，值域，刻度0.000001度,范围-90~90度;
    int latitude;
    ///经度坐标(GPS only)，值域，刻度0.000001度,范围-180~180度;
    int longitude_gps;
    ///纬度坐标(GPS only)，值域，刻度0.000001度,范围-90~90度;
    int latitude_gps;
    ///航向角值，值域，0～36000，角度刻度为0.01°
    unsigned short int heading;
    ///俯仰角值，值域，-9000～9000，角度刻度为0.01°
    short int pitch;
    ///倾斜角值，值域，-9000～9000，角度刻度为0.01°
    short int roll;
    ///无人车全局X轴向速度(东)，值域，-10000～10000，坐标刻度为0.01m/s
    short int global_vx;
    ///无人车全局Y轴向速度(北)，值域，-10000～10000，坐标刻度为0.01m/s
    short int global_vy;
    ///无人车全局H轴向速度(天)，值域，-10000～10000，坐标刻度为0.01m/s
    short int global_vz;
    ///航向角速度，值域，-10000～10000,坐标刻度为0.01°/s
    short int global_wx;
    ///俯仰角速度，值域，-10000～10000,坐标刻度为0.01°/s
    short int global_wy;
    ///倾斜角速度，值域，-10000～10000,坐标刻度为0.01°/s
    short int global_wz;
    /// 本次里程，值域，0～99999999，坐标刻度为0.1m
    unsigned int mileage;
    /// 1-valid, 0-non-valid
    bool valid_sts;
    ///0~10000%, 坐标刻度为0.01%
    unsigned char belief;

};

// Declare NML format function
extern int P_DWDX_INFOFormat(NMLTYPE, void *, CMS *);

#endif 	// P_DWDX_INFON_HH
