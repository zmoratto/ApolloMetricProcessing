// Reads SURF files
#include "surf_io.h"

#include <vw/Core.h>
#include <fstream>;

using namespace vw;
using namespace vw::ip;

std::vector<InterestPoint> read_surf_file( std::string surf_file ) {
  std::vector<InterestPoint> ip;

  // This code is for the most part a straight rip from SURF's orginal
  // We just use our own data type for the interest point.
  std::ifstream ipfile( surf_file.c_str() );
  if (!ipfile.is_open() )
    vw_throw( IOErr() << "Failed to open \"" << surf_file << "\" as SURF file.");

  // Load the file header
  unsigned count;
  int vlen; // Length of descriptor

  ipfile >> vlen >> count;

  // Load the interest points
  for ( unsigned n = 0; n < count; n++ ) {
    // Circular regions with diameter
    float x, y, a, b, c;
    
    // Read in region data, though not needed
    ipfile >> x >> y >> a >> b >> c;

    if ( a != c )
      std::cout << "ac error\n";
    if ( b != 0 )
      std::cout << "b error\n";
    
    float det = sqrt((a-c)*(a-c) + 4.0*b*b);
    float e1 = 0.5*(a+c + det);
    float e2 = 0.5*(a+c - det);
    float l1 = (1.0/sqrt(e1));
    float l2 = (1.0/sqrt(e2));
    float sc = sqrt( l1*l2 );
    float scale = sc/2.5;

    //float space;
    //ipfile >> space; // I'm not sure why I have to add it and they
		     // don't

    int laplace; // Sign of feature
    ipfile >> laplace;

    InterestPoint temp(x,y,scale,det,0.0,laplace>0);
    
    temp.descriptor.set_size(vlen-1);
    for (unsigned j = 0; j < vlen-1; j++) {
      ipfile >> temp.descriptor[j];
    }

    ip.push_back(temp);
  }
  
  ipfile.close();
  return ip;
}
