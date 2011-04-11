#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/IsisIO.h>
#include <asp/Core/Macros.h>

using namespace vw;
using namespace vw::camera;

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
namespace po = boost::program_options;
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

int main( int argc, char** argv ) {

  try {

    std::string left_cube, right_cube;

    po::options_description general_options("Options");
    general_options.add_options()
      ("help,h", "Display this help message");

    po::options_description hidden_options("");

    hidden_options.add_options()
      ("left-image", po::value(&left_cube))
      ("right-image", po::value(&right_cube));

    po::options_description options("Allowed Options");
    options.add(general_options).add(hidden_options);

    po::positional_options_description p;
    p.add("left-image", 1);
    p.add("right-image", 1);

    std::ostringstream usage;
    usage << "Usage: " << argv[0] << " [options] <left image> <right image>...\n\n";
    usage << general_options << std::endl;

    po::variables_map vm;
    try {
      po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
      po::notify( vm );
    } catch (po::error &e) {
      std::cout << "An error occured while parsing command line arguments.\n";
      std::cout << "\t" << e.what() << "\n\n";
      std::cout << usage.str();
      return 1;
    }

    if( vm.count("help") ) {
      vw_out() << usage.str();
      return 1;
    }

    if ( left_cube.empty() || right_cube.empty() ) {
      vw_out() << "Error: Must specify both input files!\n\n";
      vw_out() << usage.str();
      return 1;
    }

    // Replace extensions to cube files
    left_cube = fs::path(left_cube).replace_extension("cub").string();
    right_cube = fs::path(right_cube).replace_extension("cub").string();

    typedef std::pair<boost::shared_ptr<CameraModel>,Vector2i> CameraData;
    CameraData left = load_camera( left_cube );
    CameraData right = load_camera( right_cube );

    std::vector<Vector3> ip1;
    std::vector<Vector3> ip2;

    cartography::Datum datum("D_MOON");

    // Projecting forward 25
    for ( int32 i = 0; i < 5; i++ ) {
      int32 si = i * left.second[0]/5;
      for ( int32 j = 0; j < 5; j++ ) {
        int32 sj = j * left.second[1]/5;

        // Create XYZ point against moon
        Vector3 ccenter = left.first->camera_center( Vector2(si,sj) );
        Vector3 cpoint  = left.first->pixel_to_vector( Vector2(si,sj) );
        double radius_2 = datum.semi_major_axis() *
          datum.semi_major_axis();
        double alpha = -dot_prod(ccenter, cpoint );
        Vector3 projection = ccenter + alpha*cpoint;
        if ( norm_2_sqr(projection) > radius_2 ) {
          // did not intersect
          continue;
        }

        alpha -= sqrt( radius_2 -
                       norm_2_sqr(projection) );
        Vector3 intersection = ccenter + alpha * cpoint;

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

        // Create XYZ point against moon
        Vector3 ccenter = right.first->camera_center( Vector2(si,sj) );
        Vector3 cpoint  = right.first->pixel_to_vector( Vector2(si,sj) );
        double radius_2 = datum.semi_major_axis() *
          datum.semi_major_axis();
        double alpha = -dot_prod(ccenter, cpoint );
        Vector3 projection = ccenter + alpha*cpoint;
        if ( norm_2_sqr(projection) > radius_2 ) {
          // did not intersect
          continue;
        }

        alpha -= sqrt( radius_2 -
                       norm_2_sqr(projection) );
        Vector3 intersection = ccenter + alpha * cpoint;

        Vector2 other = left.first->point_to_pixel( intersection );
        if ( !BBox2i(0,0,left.second[0],left.second[1]).contains(other) )
          continue;
        ip2.push_back( Vector3(si,sj,1) );
        ip1.push_back( Vector3(other[0],other[1],1) );
      }
    }

    typedef math::HomographyFittingFunctor hfit_func;
    typedef math::AffineFittingFunctor afit_func;
    Matrix<double> affine = afit_func()(ip1,ip2);
    Matrix<double> homogrphy = hfit_func()(ip1,ip2,affine);
    std::cout << "M: " << homogrphy << "\n";
  } ASP_STANDARD_CATCHES;

  return 0;
}
