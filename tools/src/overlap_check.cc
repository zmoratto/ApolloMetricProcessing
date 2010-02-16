// Overlap Check .cpp

#include "overlap_check.h"

// Std header
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>

// Vision Workbench
#include <vw/Math.h>

// Isis Headers
#include <Cube.h>
#include <Camera.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <Pvl.h>
#include <SerialNumber.h>

using namespace vw;

// Push back the LatLong vertices that define the image
void points ( Isis::Camera* camera, std::list<Vector2>& points ) {
  camera->SetImage(1,1);
  points.push_back( Vector2( camera->UniversalLatitude(),
                             camera->UniversalLongitude() ));
  camera->SetImage(1,camera->Lines());
  points.push_back( Vector2( camera->UniversalLatitude(),
                             camera->UniversalLongitude() ));
  camera->SetImage(camera->Samples(),camera->Lines());
  points.push_back( Vector2( camera->UniversalLatitude(),
                             camera->UniversalLongitude() ));
  camera->SetImage(camera->Samples(),1);
  points.push_back( Vector2( camera->UniversalLatitude(),
                             camera->UniversalLongitude() ));
}

void print_list( std::list<Vector2>& list ) {
  std::cout << "Begin\n";
  for ( std::list<Vector2>::iterator it = list.begin();
        it != list.end(); it++ )
    std::cout << "\t" << *it << std::endl;
  std::cout << "End\n";
}

// Calculate delta (parametric value for line b that represents the
// intersection of line a and b )
double calc_delta( Vector2& a1, Vector2& a2,
                   Vector2& b1, Vector2& b2 ) {
  double value = (a2.x()-a1.x())*(a1.y()-b1.y())-(a2.y()-a1.y())*(a1.x()-b1.x());
  value /= (b2.y()-b1.y())*(a2.x()-a1.x())-(b2.x()-b1.x())*(a2.y()-a1.y());
  return value;
}

bool test_rightof( Vector2& p1, Vector2& p2, Vector2& test ) {
  Vector3 line;
  subvector(line,0,2) = p2-p1;
  Vector3 point;
  subvector(point,0,2) = test-p1;
  Vector3 out = cross_prod(line,point);
  return out.z() <= 0.0;
}

double triangle_area( Vector2& p1, Vector2& p2, Vector2& p3 ) {
  Vector3 u;
  subvector(u,0,2) = p2-p1;
  Vector3 v;
  subvector(v,0,2) = p3-p1;
  double value = abs(norm_2(cross_prod(u,v)));
  value /= 2;
  return value;
}

// Produce clipping polygon
std::list<Vector2> clip( std::list<Vector2> clipping,
                         std::list<Vector2> poly ) {
  typedef std::list<Vector2> plist;
  typedef plist::iterator piter;
  plist intersect;
  for ( piter it = poly.begin(); it != poly.end(); it++ )
    intersect.push_back( *it );

  // Check for programmer error
  if ( clipping.size() < 3 || intersect.size() < 3 ) {
    std::cout << "Clip error.\n Input too small\n";
    return intersect;
  }

  for ( piter edge = clipping.begin();
        edge != clipping.end(); edge++ ) {
    piter edge_next = edge;
    edge_next++;
    if ( edge_next == clipping.end() )
      edge_next = clipping.begin();

    // A. iterate, fine starting point inside of edge
    piter starting_spot = intersect.end();
    for ( piter i = intersect.begin(); i != intersect.end(); i++ ) {
      if ( test_rightof(*edge,*edge_next,*i)) {
        starting_spot = i;
        break;
      }
    }
    if ( starting_spot == intersect.end() || intersect.size() < 3 ) {
      intersect.clear();
      return intersect;
    }

    // B. iterate, around intersect
    piter i = starting_spot;
    piter ni = starting_spot;
    ni++;
    bool just_added = false;
    bool added_last_time = false;
    do {
      added_last_time = just_added;
      just_added = false;

      // B.1 if segment ahead crosses boundary, add_point
      if ( test_rightof( *edge, *edge_next, *i ) ^
           test_rightof( *edge, *edge_next, *ni ) ) {
        // Find intersection
        double delta = calc_delta( *edge, *edge_next,
                                   *i, *ni );
        Vector2 npoint = (*i) + delta*((*ni)-(*i));
        if ( npoint != *i ) {
          ni = intersect.insert(ni,npoint);
          just_added = true;
        }
      }

      // B.2 if current is above clipping plane, delete it.
      if ( !test_rightof( *edge, *edge_next, *i ) && !added_last_time ) {
        i = intersect.erase(i);
        i--;
      }

      ni++;
      i++;

      // Looping stuff
      if ( ni == intersect.end() )
        ni = intersect.begin();
      else if ( i == intersect.end() )
        i = intersect.begin();

    } while ( i != starting_spot );
  }

  return intersect;
}

// ONLY thing visible to the user
//--------------------------------------------
double percent_overlap( std::string& l_cube,
                        std::string& r_cube ) {
  // Opening both cube files
  Isis::Cube* l_cube_ptr = new Isis::Cube;
  Isis::Cube* r_cube_ptr = new Isis::Cube;
  l_cube_ptr->Open(l_cube);
  r_cube_ptr->Open(r_cube);

  // Getting cameras
  Isis::Camera* l_cam = static_cast<Isis::Camera*>(l_cube_ptr->Camera());
  Isis::Camera* r_cam = static_cast<Isis::Camera*>(r_cube_ptr->Camera());

  // Edge points
  typedef std::list<Vector2> plist;
  plist l_points;
  plist r_points;
  points( l_cam, l_points );
  points( r_cam, r_points );

  // Checking for wrapping around 0->360
  bool long_low = false;
  bool long_high = false;
  for ( plist::iterator it = l_points.begin();
        it != l_points.end(); it++ ) {
    if (it->y() < 180 )
      long_low |= true;
    else
      long_high |= true;
  }
  for ( plist::iterator it = r_points.begin();
        it != r_points.end(); it++ ) {
    if (it->y() < 180 )
      long_low |= true;
    else
      long_high |= true;
  }
  if ( long_low && long_high ) {
    for ( plist::iterator it = l_points.begin();
          it != l_points.end(); it++ ) {
      if (it->y() < 180)
        it->y() += 180;
      else
        it->y() -= 180;
    }
    for ( plist::iterator it = r_points.begin();
          it != r_points.end(); it++ ) {
      if (it->y() < 180)
        it->y() += 180;
      else
        it->y() -= 180;
    }
  }

  // Double checking to make sure the long range is still no too
  // large. This should block false positives.
  double min_long = 360;
  double max_long = 0;
  for ( plist::iterator it = l_points.begin();
        it != l_points.end(); it++ )
    if ( it->y() < min_long )
      min_long = it->y();
    else if ( it->y() > max_long )
      max_long = it->y();
  for ( plist::iterator it = r_points.begin();
        it != r_points.end(); it++ )
    if ( it->y() < min_long )
      min_long = it->y();
    else if ( it->y() > max_long )
      max_long = it->y();
  if ( (max_long - min_long) > 180 ) // Clipping just fails with this
    return 0;

  plist clipped = clip(l_points, r_points);

  // Overlap of control region
  double control_area = 0;
  {
    plist::iterator starting_spot = l_points.begin();
    plist::iterator index = starting_spot;
    index++;
    plist::iterator nindex = index;
    nindex++;
    while ( nindex != l_points.end() ) {
      control_area += triangle_area( *starting_spot,
                                     *index, *nindex );
      index++;
      nindex++;
    }
  }
  // Overlap of clipped region
  double clipped_area = 0;
  {
    plist::iterator starting_spot = clipped.begin();
    plist::iterator index = starting_spot;
    index++;
    plist::iterator nindex = index;
    nindex++;
    while ( nindex != clipped.end() ) {
      clipped_area += triangle_area( *starting_spot,
                                     *index, *nindex );
      index++;
      nindex++;
    }
  }

  double overlap = 100*clipped_area/control_area;
  return overlap;
}
