#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include "Kriging.h"
#include <vw/Stereo.h>
#include <asp/IsisIO.h>
#include <vw/Cartography.h>
#include <vw/Stereo/CorrelatorView.h>
#include <sstream>

using namespace vw;
using namespace vw::ip;
using namespace vw::camera;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

std::pair<boost::shared_ptr<CameraModel>, Vector2i>
load_camera( std::string const& cube ) {
  std::pair<boost::shared_ptr<CameraModel>, Vector2i> result;

  std::string adjust_file =
    fs::path( cube ).replace_extension("isis_adjust").string();
  if ( fs::exists( adjust_file ) ) {
    vw_out() << "Loading \"" << adjust_file << "\"\n";
    std::ifstream input( adjust_file.c_str() );
    boost::shared_ptr<asp::BaseEquation> posF = asp::read_equation(input);
    boost::shared_ptr<asp::BaseEquation> poseF = asp::read_equation(input);
    input.close();
    boost::shared_ptr<IsisAdjustCameraModel> model( new IsisAdjustCameraModel( cube, posF, poseF ) );
    result.second[0] = model->samples();
    result.second[1] = model->lines();
    result.first = model;
  } else {
    vw_out() << "Loading \"" << cube << "\"\n";
    boost::shared_ptr<IsisCameraModel> model( new IsisCameraModel( cube ) );
    result.second[0] = model->samples();
    result.second[1] = model->lines();
    result.first = model;
  }
  return result;
}

Vector3 datum_intersection( cartography::Datum const& datum,
                            boost::shared_ptr<CameraModel> model,
                            Vector2 const& pix ) {
  // Create XYZ point against moon
  Vector3 ccenter = model->camera_center( pix );
  Vector3 cpoint  = model->pixel_to_vector( pix );
  double radius_2 = datum.semi_major_axis() *
    datum.semi_major_axis();
  double alpha = -dot_prod(ccenter, cpoint );
  Vector3 projection = ccenter + alpha*cpoint;
  if ( norm_2_sqr(projection) > radius_2 ) {
    // did not intersect
    return Vector3();
  }

  alpha -= sqrt( radius_2 -
                 norm_2_sqr(projection) );
  return ccenter + alpha * cpoint;
}

Matrix<double> predict_homography( std::pair<boost::shared_ptr<CameraModel>, Vector2i> left,
                                   std::pair<boost::shared_ptr<CameraModel>, Vector2i> right ) {
  std::vector<Vector3> ip1;
  std::vector<Vector3> ip2;

  cartography::Datum datum("D_MOON");
  std::cout << "Left size: " << left.second << "\n";
  std::cout << "Rgih size: " << right.second << "\n";

  // Projecting forward 25
  for ( int32 i = 0; i < 5; i++ ) {
    int32 si = i * left.second[0]/5;
    for ( int32 j = 0; j < 5; j++ ) {
      int32 sj = j * left.second[1]/5;

      Vector3 intersection =
        datum_intersection( datum, left.first, Vector2(si,sj) );
      if ( intersection == Vector3() )
        continue;

      Vector2 other = right.first->point_to_pixel( intersection );
      if ( !BBox2i(0,0,right.second[0],right.second[1]).contains(other) )
        continue;
      ip1.push_back( Vector3(si,sj,1) );
      ip2.push_back( Vector3(other[0],other[1],1) );
    }
  }

  // Projecting backward 25
  for ( uint32 i = 0; i < 5; i++ ) {
    int32 si = i * right.second[0]/5;
    for ( uint32 j = 0; j < 5; j++ ) {
      int32 sj = j * right.second[1]/5;

      Vector3 intersection =
        datum_intersection( datum, right.first, Vector2(si,sj) );
      if ( intersection == Vector3() )
        continue;

      Vector2 other = left.first->point_to_pixel( intersection );
      if ( !BBox2i(0,0,left.second[0],left.second[1]).contains(other) )
        continue;
      ip2.push_back( Vector3(si,sj,1) );
      ip1.push_back( Vector3(other[0],other[1],1) );
    }
  }

  std::vector<Vector3> copy( ip1 );
  BOOST_FOREACH( Vector3 const& i, copy ) {
    Vector3 intersection = datum_intersection( datum, left.first, Vector2(i[0]+5,i[1]) );
    if ( intersection != Vector3() ) {
      Vector2 other = right.first->point_to_pixel( intersection );
      if ( BBox2i(0,0,right.second[0],right.second[1]).contains(other) ) {
        ip1.push_back( i + Vector3(5,0,0) );
        ip2.push_back( Vector3(other[0],other[1],1) );
      }
    }

    intersection = datum_intersection( datum, left.first, Vector2(i[0],i[1]+5) );
    if ( intersection != Vector3() ) {
      Vector2 other = right.first->point_to_pixel( intersection );
      if ( BBox2i(0,0,right.second[0],right.second[1]).contains(other) ) {
        ip1.push_back( i + Vector3(0,5,0) );
        ip2.push_back( Vector3(other[0],other[1],1) );
      }
    }

  }

  // Pick out 4
  typedef math::HomographyFittingFunctor hfit_func;
  math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric> ransac( hfit_func(), math::InterestPointErrorMetric(), 10 );
  return ransac(ip1,ip2);
}

int main(int argc, char** argv) {
  std::string left, right;
  double sigma;
  int kernel;

  po::options_description general_options("Options");
  general_options.add_options()
    ("sigma,s", po::value(&sigma)->default_value(50), "Search region in pixels")
    ("kernel,k", po::value(&kernel)->default_value(19), "Kernel size")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("left-image", po::value(&left))
    ("right-image", po::value(&right));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("left-image", 1);
  p.add("right-image", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <filenames>...\n\n";
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_out() << "An error occured while parsing command line arguments.\n";
    vw_out() << "\t" << e.what() << "\n\n";
    vw_out() << usage.str();
    return 1;
  }

  if( vm.count("help") ) {
    vw_out() << usage.str();
    return 1;
  }

  if ( left.empty() || right.empty() ) {
    vw_out() << "Error: Must specify at least two input files!" << std::endl << std::endl;
    vw_out() << usage.str();
    return 1;
  }

  // Load in camera models
  Matrix<double> homography;
  if ( boost::contains(left,"wac") ) {
    homography = math::identity_matrix<3>();
  } else {
    std::string left_cube = fs::path(left).replace_extension("cub").string();
    std::string right_cube = fs::path(right).replace_extension("cub").string();
    typedef std::pair<boost::shared_ptr<CameraModel>, Vector2i> CameraData;
    CameraData left_cam =  load_camera( left_cube );
    CameraData right_cam = load_camera( right_cube );
    homography = predict_homography( left_cam, right_cam );
  }
  std::cout << "Homography: " << homography << "\n";

  // Transform left image
  DiskImageView<uint8> left_image( left ), right_image( right );
  DiskCacheImageView<uint8> left_transformed( transform( left_image,
                                                         HomographyTransform( homography )));
  // Loading interest points
  typedef std::vector< ip::InterestPoint > IPVector;
  IPVector left_ip, right_ip;
  std::string prefix =
    fs::path(left).replace_extension("").string() + "__" +
    fs::path(right).replace_extension("").string();
  std::string output_filename = prefix + ".match";
  read_binary_match_file( output_filename, left_ip, right_ip );

  // Loop over every IP and refine
  for ( size_t i = 0; i < left_ip.size(); i++ ) {

    Vector3 left_t_ip = homography*Vector3( left_ip[i].x, left_ip[i].y, 1 );
    left_t_ip /= left_t_ip[2];

    BBox2i lbbox(left_t_ip[0],left_t_ip[1],1,1);
    BBox2i rbbox(right_ip[i].ix,right_ip[i].iy,1,1);
    lbbox.expand(sigma+kernel);
    rbbox.expand(sigma+kernel);;
    Vector2 starting_right( right_ip[i].x, right_ip[i].y );

    ImageView<uint8> left_crop = crop( edge_extend(left_transformed, ZeroEdgeExtension()), lbbox );
    ImageView<uint8> right_crop = crop( edge_extend(right_image, ZeroEdgeExtension()), rbbox );
    ImageView<uint8> left_crop_mask =
      crop( transform( constant_view(uint8(255), left_image.cols(),
                                     left_image.rows()),
                       HomographyTransform( homography )),
            lbbox );
    ImageView<uint8> right_crop_mask =
      constant_view( uint8(255), rbbox.width(),
                     rbbox.height() );

    /*
    std::ostringstream ostr;
    ostr << i << "_";
    write_image(ostr.str()+"l.tif", left_crop );
    write_image(ostr.str()+"r.tif", right_crop );
    write_image(ostr.str()+"lm.tif", left_crop_mask );
    write_image(ostr.str()+"rm.tif", right_crop_mask );
    */

    typedef stereo::CorrelatorView<uint8,uint8,stereo::LogStereoPreprocessingFilter> LogCorrView;
    LogCorrView corr_view( left_crop, right_crop, left_crop_mask, right_crop_mask,
                           stereo::LogStereoPreprocessingFilter(1.4), false );
    corr_view.set_search_range( BBox2(-sigma/2,-sigma/2,sigma,sigma) );
    corr_view.set_kernel_size(Vector2i(kernel,kernel));
    corr_view.set_correlator_options( 2, stereo::NORM_XCORR_CORRELATOR );

    ImageView<PixelMask<Vector2f> > idisparity = corr_view;
    PixelMask<Vector2f> change = idisparity( left_t_ip[0] - lbbox.min()[0],
                                             left_t_ip[1] - lbbox.min()[1] );

    if ( is_valid( change ) ) {
      std::cout << "Change: " << change << "\n";
      right_ip[i].x += change[0];
      right_ip[i].ix += change[0];
      right_ip[i].y += change[1];
      right_ip[i].iy += change[1];
    }
  }

  write_binary_match_file( output_filename, left_ip, right_ip );

  return 0;
}
