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
    boost::polygon::construct<Point>(1145,599),
    boost::polygon::construct<Point>(1126,570),
    boost::polygon::construct<Point>(1145,570)
  };
  Point BOT_FIDUCIAL[] = {
    boost::polygon::construct<Point>(545,1145),
    boost::polygon::construct<Point>(573,1126),
    boost::polygon::construct<Point>(573,1145)
  };
  Point LENS_CAP[] = {
    boost::polygon::construct<Point>(1030,0),
    boost::polygon::construct<Point>(1010,52),
    boost::polygon::construct<Point>(994,243),
    boost::polygon::construct<Point>(881,491),
    boost::polygon::construct<Point>(855,571),
    boost::polygon::construct<Point>(846,664),
    boost::polygon::construct<Point>(862,761),
    boost::polygon::construct<Point>(896,827),
    boost::polygon::construct<Point>(862,933),
    boost::polygon::construct<Point>(868,987),
    boost::polygon::construct<Point>(911,1058),
    boost::polygon::construct<Point>(1022,1145),
    boost::polygon::construct<Point>(1145,1145),
    boost::polygon::construct<Point>(1145,0)
  };
  Point ANTENNA[] = {
    boost::polygon::construct<Point>(1145,524),
    boost::polygon::construct<Point>(781,535),
    boost::polygon::construct<Point>(765,547),
    boost::polygon::construct<Point>(746,575),
    boost::polygon::construct<Point>(747,606),
    boost::polygon::construct<Point>(762,619),
    boost::polygon::construct<Point>(1145,687)
  };

  struct ApolloShapes {
    Polygon left_fiducial, top_fiducial, right_fiducial, bot_fiducial;
    Polygon lens_cap, antenna;

    ApolloShapes() {
      boost::polygon::set_points(left_fiducial, LEFT_FIDUCIAL, LEFT_FIDUCIAL+3);
      boost::polygon::set_points(top_fiducial,  TOP_FIDUCIAL,  TOP_FIDUCIAL+3);
      boost::polygon::set_points(right_fiducial,RIGHT_FIDUCIAL,RIGHT_FIDUCIAL+3);
      boost::polygon::set_points(bot_fiducial,  BOT_FIDUCIAL,  BOT_FIDUCIAL+3);
      boost::polygon::set_points(lens_cap,      LENS_CAP,      LENS_CAP+14);
      boost::polygon::set_points(antenna,       ANTENNA,       ANTENNA+7);
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
