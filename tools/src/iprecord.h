
#ifndef __IP_RECORD_H__
#define __IP_RECORD_H__

#include <vw/InterestPoint/InterestData.h>

namespace vw {

  // These should probably be public to the users.

  inline void write_ip_record(std::ostream &f, InterestPoint const& p) {
    f.write((char*)&(p.x), sizeof(p.x));
    f.write((char*)&(p.y), sizeof(p.y));
    f.write((char*)&(p.ix), sizeof(p.ix));
    f.write((char*)&(p.iy), sizeof(p.iy));
    f.write((char*)&(p.orientation), sizeof(p.orientation));
    f.write((char*)&(p.scale), sizeof(p.scale));
    f.write((char*)&(p.interest), sizeof(p.interest));
    f.write((char*)&(p.polarity), sizeof(p.polarity));
    f.write((char*)&(p.octave), sizeof(p.octave));
    f.write((char*)&(p.scale_lvl), sizeof(p.scale_lvl));
    int size = p.size();
    f.write((char*)(&size), sizeof(int));
    for (unsigned i = 0; i < p.descriptor.size(); ++i)
      f.write((char*)&(p.descriptor[i]), sizeof(p.descriptor[i]));
  }

  inline InterestPoint read_ip_record(std::istream &f) {
    InterestPoint ip;
    f.read((char*)&(ip.x), sizeof(ip.x));
    f.read((char*)&(ip.y), sizeof(ip.y));
    f.read((char*)&(ip.ix), sizeof(ip.ix));
    f.read((char*)&(ip.iy), sizeof(ip.iy));
    f.read((char*)&(ip.orientation), sizeof(ip.orientation));
    f.read((char*)&(ip.scale), sizeof(ip.scale));
    f.read((char*)&(ip.interest), sizeof(ip.interest));
    f.read((char*)&(ip.polarity), sizeof(ip.polarity));
    f.read((char*)&(ip.octave), sizeof(ip.octave));
    f.read((char*)&(ip.scale_lvl), sizeof(ip.scale_lvl));

    int size;
    f.read((char*)&(size), sizeof(ip.descriptor.size()));
    ip.descriptor = Vector<double>(size);
    for (int i = 0; i < size; ++i)
      f.read((char*)&(ip.descriptor[i]), sizeof(ip.descriptor[i]));
    return ip;
  }

}

#endif
