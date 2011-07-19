#ifndef __VW_APOLLO_SHAPES_H__
#define __VW_APOLLO_SHAPES_H__

#include <boost/lexical_cast.hpp>
#include <boost/polygon/polygon.hpp>

namespace vw {

  typedef boost::polygon::polygon_data<float> Polygon;
  typedef boost::polygon::polygon_traits<Polygon>::point_type Point;
  Point LEFT_FIDUCIAL[] = {
    boost::polygon::construct<Point>(0,545),
    boost::polygon::construct<Point>(17,573),
    boost::polygon::construct<Point>(0,574)
  };
  Point TOP_FIDUCIAL[] = {
    boost::polygon::construct<Point>(574,0),
    boost::polygon::construct<Point>(573,17),
    boost::polygon::construct<Point>(544,0)
  };
  Point RIGHT_FIDUCIAL[] = {
    boost::polygon::construct<Point>(1144,599),
    boost::polygon::construct<Point>(1126,570),
    boost::polygon::construct<Point>(1144,570)
  };
  Point BOT_FIDUCIAL[] = {
    boost::polygon::construct<Point>(545,1144),
    boost::polygon::construct<Point>(573,1126),
    boost::polygon::construct<Point>(573,1144)
  };
  Point LENS_CAP[] = {
    boost::polygon::construct<Point>(1144,0),
    boost::polygon::construct<Point>(1144,1144),
    boost::polygon::construct<Point>(1082,1144),
    boost::polygon::construct<Point>(899,968),
    boost::polygon::construct<Point>(902,901),
    boost::polygon::construct<Point>(932,831),
    boost::polygon::construct<Point>(885,721),
    boost::polygon::construct<Point>(873,625),
    boost::polygon::construct<Point>(1048,122),
    boost::polygon::construct<Point>(1055,0)
  };
  Point ANTENNA[] = {
    boost::polygon::construct<Point>(1144,631),
    boost::polygon::construct<Point>(779,601),
    boost::polygon::construct<Point>(780,564),
    boost::polygon::construct<Point>(1144,570)
  };

  struct ApolloShapes {
    Polygon left_fiducial, top_fiducial, right_fiducial, bot_fiducial;
    Polygon lens_cap, antenna;

    ApolloShapes() {
      boost::polygon::set_points(left_fiducial, LEFT_FIDUCIAL, LEFT_FIDUCIAL+3);
      boost::polygon::set_points(top_fiducial,  TOP_FIDUCIAL,  TOP_FIDUCIAL+3);
      boost::polygon::set_points(right_fiducial,RIGHT_FIDUCIAL,RIGHT_FIDUCIAL+3);
      boost::polygon::set_points(bot_fiducial,  BOT_FIDUCIAL,  BOT_FIDUCIAL+3);
      boost::polygon::set_points(lens_cap,      LENS_CAP,      LENS_CAP+10);
      boost::polygon::set_points(antenna,       ANTENNA,       ANTENNA+4);
    }

    inline bool in_fiducial( Vector2f const& p ) const {
      Point pc = boost::polygon::construct<Point>(p[0],p[1]);
      return boost::polygon::contains(left_fiducial, pc) ||
        boost::polygon::contains(top_fiducial, pc) ||
        boost::polygon::contains(right_fiducial, pc) ||
        boost::polygon::contains(bot_fiducial, pc);
    }

    inline bool in_lens_cap( Vector2f const& p ) const {
      return boost::polygon::contains(lens_cap, boost::polygon::construct<Point>(p[0],p[1]));
    }

    inline bool in_antenna( Vector2f const& p ) const {
      return boost::polygon::contains(antenna, boost::polygon::construct<Point>(p[0],p[1]));
    }
  };

  int32 extract_camera_number( std::string filename ) {
    size_t s_idx = filename.find("AS");
    std::string mission_string = filename.substr(s_idx+2,2);
    std::string image_string   = filename.substr(s_idx+7,4);
    return boost::lexical_cast<int32>(mission_string)*10000 + boost::lexical_cast<int32>(image_string);
  }

}

#endif//__VW_APOLLO_SHAPES_H__
