#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint.h>

#include <asp/IsisIO.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

#include <boost/polygon/polygon.hpp>
namespace poly = boost::polygon;

#include "../src/camera_fitting.h"

using namespace vw;

typedef poly::polygon_data<float> Polygon;
typedef poly::polygon_traits<Polygon>::point_type Point;
Point LEFT_FIDUCIAL[] = {
  poly::construct<Point>(0,545),
  poly::construct<Point>(17,573),
  poly::construct<Point>(0,574)
};
Point TOP_FIDUCIAL[] = {
  poly::construct<Point>(574,0),
  poly::construct<Point>(573,17),
  poly::construct<Point>(544,0)
};
Point RIGHT_FIDUCIAL[] = {
  poly::construct<Point>(1144,599),
  poly::construct<Point>(1126,570),
  poly::construct<Point>(1144,570)
};
Point BOT_FIDUCIAL[] = {
  poly::construct<Point>(545,1144),
  poly::construct<Point>(573,1126),
  poly::construct<Point>(573,1144)
};
Point LENS_CAP[] = {
  poly::construct<Point>(1144,0),
  poly::construct<Point>(1144,1144),
  poly::construct<Point>(1082,1144),
  poly::construct<Point>(899,968),
  poly::construct<Point>(902,901),
  poly::construct<Point>(932,831),
  poly::construct<Point>(885,721),
  poly::construct<Point>(873,625),
  poly::construct<Point>(1048,122),
  poly::construct<Point>(1055,0)
};
Point ANTENNA[] = {
  poly::construct<Point>(1144,631),
  poly::construct<Point>(779,601),
  poly::construct<Point>(780,564),
  poly::construct<Point>(1144,570)
};

struct ApolloShapes {
  Polygon left_fiducial, top_fiducial, right_fiducial, bot_fiducial;
  Polygon lens_cap, antenna;

  ApolloShapes() {
    poly::set_points(left_fiducial, LEFT_FIDUCIAL, LEFT_FIDUCIAL+3);
    poly::set_points(top_fiducial,  TOP_FIDUCIAL,  TOP_FIDUCIAL+3);
    poly::set_points(right_fiducial,RIGHT_FIDUCIAL,RIGHT_FIDUCIAL+3);
    poly::set_points(bot_fiducial,  BOT_FIDUCIAL,  BOT_FIDUCIAL+3);
    poly::set_points(lens_cap,      LENS_CAP,      LENS_CAP+10);
    poly::set_points(antenna,       ANTENNA,       ANTENNA+4);
  }

  inline bool in_fiducial( Vector2f const& p ) const {
    Point pc = poly::construct<Point>(p[0],p[1]);
    return poly::contains(left_fiducial, pc) ||
      poly::contains(top_fiducial, pc) ||
      poly::contains(right_fiducial, pc) ||
      poly::contains(bot_fiducial, pc);
  }

  inline bool in_lens_cap( Vector2f const& p ) const {
    return poly::contains(lens_cap, poly::construct<Point>(p[0],p[1]));
  }

  inline bool in_antenna( Vector2f const& p ) const {
    return poly::contains(antenna, poly::construct<Point>(p[0],p[1]));
  }
};

int32 extract_camera_number( std::string filename ) {
  size_t s_idx = filename.find("AS");
  std::string mission_string = filename.substr(s_idx+2,2);
  std::string image_string   = filename.substr(s_idx+7,4);
  return boost::lexical_cast<int32>(mission_string)*10000 + boost::lexical_cast<int32>(image_string);
}

int main( int argc, char* argv[] ) {

  std::string cam_file, image_file;
  po::options_description general_options("Options");
  general_options.add_options()
    ("cam-file", po::value(&cam_file), "A file listing the input cube files.")
    ("image-file", po::value(&image_file), "Input control network.")
    ("help,h", "Display this help message");

  po::positional_options_description p;
  p.add("cam-file", 1);
  p.add("image-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(general_options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <cam file> <image file> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( cam_file.empty() || image_file.empty() ) {
    vw_out() << "ERROR! Require input camera model and image.\n";
    vw_out() << "\n" << usage.str() << "\n";
    return 1;
  }

  // Loading camera model and image
  DiskImageView<PixelGray<float> > image( image_file );
  Vector2f image_size( image.cols(), image.rows() );
  boost::shared_ptr<camera::CameraModel> camera;
  if ( fs::extension( cam_file ) == ".isis_adjust" ) {
    typedef boost::shared_ptr<asp::BaseEquation> EqnPtr;
    std::ifstream input( cam_file.c_str() );
    EqnPtr posF  = asp::read_equation(input);
    EqnPtr poseF = asp::read_equation(input);
    input.close();
    camera.reset( new camera::IsisAdjustCameraModel( fs::change_extension(cam_file,
                                                                          ".cub").string(),
                                                     posF, poseF ) );
  } else {
    camera.reset( new camera::IsisCameraModel( cam_file ) );
  }

  // Working out rotation and producing rotated image
  cartography::GeoReference georef( cartography::Datum("D_MOON"),
                                    math::identity_matrix<3>() );
  bool working;
  Vector2 l_direction =
    cartography::geospatial_intersect( Vector2(), georef, camera,
                                       1, working ) -
    cartography::geospatial_intersect( Vector2(0,image.rows()-1),
                                       georef, camera, 1, working );
  if (l_direction[0] < -200)
    l_direction[0] += 360;
  if (l_direction[0] > 200 )
    l_direction[0] -= 360;
  Vector2 r_direction =
    cartography::geospatial_intersect( Vector2(image.cols()-1,0), georef,
                                       camera, 1, working ) -
    cartography::geospatial_intersect( image_size - Vector2(1,1),
                                       georef, camera, 1, working );
  if (r_direction[0] < -200)
    r_direction[0] += 360;
  if (r_direction[0] > 200)
    r_direction[0] -= 360;
  double rotate = M_PI/2 - (atan2(l_direction[1],l_direction[0]) +
                            atan2(r_direction[1],r_direction[0]) )/2;
  Vector2 pivot = (image_size - Vector2(1,1))/2;

  Vector2 extrema1 =
    RotateTransform( rotate, pivot ).forward( image_size - Vector2(1,1) ) -
    image_size + Vector2(1,1);
  Vector2 extrema2 =
    RotateTransform( rotate, pivot ).forward( Vector2() ) -
    image_size + Vector2(1,1);
  double extension =
    std::max( max( extrema1 ), max( extrema2 ) );
  CompositionTransform<TranslateTransform,RotateTransform>
    trans( TranslateTransform( extension, extension ),
           RotateTransform( rotate, pivot ) );
  DiskCacheImageView<PixelGray<float> > cache(
      crop(transform( image, trans ), 0, 0,
           extension*2+image_size[0],
           extension*2+image_size[1]), "tif",
      TerminalProgressCallback("","Caching:") );

  // OBALOG
  ip::OBALoGInterestOperator interest_operator(.035);
  ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator> detector( interest_operator,
                                                                  0 );
  ip::InterestPointList ip = detect_interest_points(cache,detector);
  vw_out() << "\tDetected IP: " << ip.size() << "\n";

  // BRIEF ( or SGRAD for the time being )
  ip::SGradDescriptorGenerator gen;
  BOOST_FOREACH( ip::InterestPoint& pt, ip ) {
    float scaling = 1.0f / pt.scale;
    ImageView<PixelGray<float> > patch =
      transform( cache,
                 AffineTransform( Matrix2x2( scaling, 0, 0, scaling ),
                                  Vector2(-scaling*pt.x+20.5,
                                          -scaling*pt.y+20.5) ),
                 42, 42 );
    pt.descriptor.set_size( 180 );
    gen.compute_descriptor( patch, pt.begin(), pt.end() );
  }

  // Transform back to orignal coordinates
  BOOST_FOREACH( ip::InterestPoint& pt, ip ) {
    Vector2f loc = trans.reverse( Vector2f(pt.x,pt.y) );
    pt.x = loc[0];
    pt.y = loc[1];
    pt.ix = loc[0];
    pt.iy = loc[1];
  }

  // Filtering out fudicials
  ApolloShapes shapes;
  BBox2f image_bbox = bounding_box(image);
  image_bbox.contract(1);
  int32 image_number = extract_camera_number( image_file );
  for ( ip::InterestPointList::iterator point = ip.begin();
        point != ip.end(); ++point ) {
    Vector2f point_loc(point->x,point->y);
    bool todelete = false;
    if ( !image_bbox.contains( point_loc ) ||
         shapes.in_fiducial( point_loc ) )
      todelete = true;
    if ( image_number == 151944 ||
         (image_number >= 170233 && image_number <= 170313) ||
         (image_number >= 171981 && image_number <= 172124) ) {
      if ( shapes.in_lens_cap( point_loc ) )
        todelete = true;
    }
    if ( (image_number >= 152093 && image_number <= 152204) ||
         (image_number >= 161145 && image_number <= 161650) ||
         (image_number >= 161897 && image_number <= 161985) ||
         (image_number >= 162156 && image_number <= 162845) ) {
      if ( shapes.in_antenna( point_loc ) )
        todelete = true;
    }
    if ( todelete ) {
      point = ip.erase(point);
      point--;
      continue;
    }
  }

  // Linearize Camera Model
  camera::PinholeModel linear_camera =
    linearize_pinhole( camera.get(), image_size );

  // Transform measurements to linearized camera models
  BOOST_FOREACH( ip::InterestPoint& point, ip ) {
    Vector2 distorted( point.x , point.y );
    Vector2 undistort =
      linear_camera.point_to_pixel(camera->camera_center( distorted ) +
                                   camera->pixel_to_vector( distorted )) -
      linear_camera.point_offset();
    point.x = undistort[0];
    point.y = undistort[1];
    point.ix = undistort[0];
    point.iy = undistort[1];
  }

  // Write NVM
  std::ofstream nvm( fs::change_extension( image_file, ".nvm").string().c_str(),
                     std::ofstream::out );
  nvm << std::setprecision(12);
  nvm << "NVM_V3_R9T\n1\n";
  Matrix3x3 crot = transpose(linear_camera.camera_pose().rotation_matrix());
  Vector3 ctrans = -(crot * linear_camera.camera_center());
  nvm << crot(0,0) << " " << crot(0,1) << " " << crot(0,2) << " "
      << crot(1,0) << " " << crot(1,1) << " " << crot(1,2) << " "
      << crot(2,0) << " " << crot(2,1) << " " << crot(2,2) << " ";
  nvm << ctrans[0] << " " << ctrans[1] << " " << ctrans[2] << " 0 0\n";
  nvm.close();

  // Write Interest Points
  vw_out() << "\tFinal IP: " << ip.size() << "\n";
  write_binary_ip_file( fs::change_extension( image_file, ".vwip" ).string(),
                        ip );

  return 0;
}
