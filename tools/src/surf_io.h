// Used to read surf file type
#ifndef __SURF_IO_H__
#define __SURF_IO_H__

#include <vw/InterestPoint/InterestData.h>
#include <vector>

std::vector<vw::ip::InterestPoint> read_surf_file( std::string surf_file );

#endif//__SURF_IO_H__
