#ifndef __RCSHEADER_HH_
#define __RCSHEADER_HH_

#include "rcs.hh"

#ifndef RCS_MAX_STR_LEN_FRAMEID
#define RCS_MAX_STR_LEN_FRAMEID 10
#endif //RCS_MAX_STR_LEN_FRAMEID

///RCS version of ROS std_msgs::Header
struct RCSHeader
{
	/// Sequence number of message header
	unsigned long seq;
	/// the stamp should be uint64_t but it is not supported in RCS, use uint32_t instead
	unsigned long stamp;
	/// char array with variable length
	DECLARE_NML_DYNAMIC_LENGTH_ARRAY(char, frame_id, RCS_MAX_STR_LEN_FRAMEID);
};

extern void nmlupdate(CMS *cms, RCSHeader *x);

#endif //__RCSHEADER_HH_
