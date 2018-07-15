/*
	 P_PointC::init(argc, argv, "velodyne_rcs_wrapper_node");
	   7   ros::NodeHandle nh("~");
		   8   std::string filename;
			   9   int r;
				  10   nh.param("nml",filename,std::string("velodyne_rcs_wrapper.nml"));
					oud_1n.hh

This C++ header file defines the NML Messages for P_POINTCLOUD_1
Template Version 1.1

MODIFICATIONS:
Tue May 08 19:28:29 CST 2018	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef P_POINTCLOUD_1N_HH
#define P_POINTCLOUD_1N_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define P_POINTCLOUD_1_MSG_TYPE 108000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

#ifndef RCS_MAX_PPS
#define RCS_MAX_PPS 2200000
#endif

#ifndef RCS_MAX_NUM_FIELDS
#define RCS_MAX_NUM_FIELDS 5
#endif

#ifndef RCS_MAX_STR_LEN_CHANNELNAME
#define RCS_MAX_STR_LEN_CHANNELNAME 20
#endif

#include "rcsheadern.hh"

///The struct for PointField corresponding to PCLPointField
struct RCSPointField
{
	unsigned long offset;
	unsigned char datatype;
	unsigned long count;
	DECLARE_NML_DYNAMIC_LENGTH_ARRAY(char, name, RCS_MAX_STR_LEN_CHANNELNAME);
};

extern void nmlupdate(CMS *cms, RCSHeader *x);
extern void nmlupdate(CMS *cms, RCSPointField *x);

// Define the NML Message Classes

class P_POINTCLOUD_1_MSG : public NMLmsg
{
public:

	//Constructor
	P_POINTCLOUD_1_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
	unsigned long height;
	unsigned long width;
	unsigned char	is_bigendian; 
	unsigned long point_step;
	unsigned long row_step;
	unsigned char	is_dense; 
	RCSHeader header;
	DECLARE_NML_DYNAMIC_LENGTH_ARRAY(RCSPointField,fields,RCS_MAX_NUM_FIELDS);
	DECLARE_NML_DYNAMIC_LENGTH_ARRAY(unsigned char,data,RCS_MAX_PPS*RCS_MAX_NUM_FIELDS);
};

// Declare NML format function
extern int P_PointCloud_1Format(NMLTYPE, void *, CMS *);

#endif 	// P_POINTCLOUD_1N_HH
