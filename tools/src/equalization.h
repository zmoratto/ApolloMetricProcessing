// This will knock off weak points where we already have a lot of
// points. This is hopefully to produce a better distribution.
#ifndef __EQUALIZATION_H__
#define __EQUALIZATION_H__

#include <vw/InterestPoint/InterestData.h>
#include <vector>

void equalization( std::vector<vw::ip::InterestPoint>& l_ip,
		   std::vector<vw::ip::InterestPoint>& r_ip,
		   int max_points );

#endif//__EQUALIZATION_H__
