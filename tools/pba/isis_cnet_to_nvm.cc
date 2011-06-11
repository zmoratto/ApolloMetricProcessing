#include <vw/Core.h>
#include <vw/BundleAdjustment.h>
#include <asp/IsisIO.h>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/convenience.hpp>

#include "../src/camera_fitting.h"
#include "../pba/nvmio.h"

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct CameraInfo {
  CameraInfo( std::string const& f,
              camera::CameraModel* i,
              camera::PinholeModel const& p ) :
    filename(f), isis_model(i), model(p) {}
  std::string filename;
  boost::shared_ptr<camera::CameraModel> isis_model; // for copy
  camera::PinholeModel model;
};

int main( int argc, char* argv[] ) {

  std::string cube_list_file, cnet_file;
  po::options_description general_options("Options");
  general_options.add_options()
    ("cube-list", po::value(&cube_list_file), "A file listing the input cube files.")
    ("cnet-file", po::value(&cnet_file), "Input control network.")
    ("help,h", "Display this help message");

  po::positional_options_description p;
  p.add("cube-list", 1);
  p.add("cnet-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(general_options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <list-file> <cnet-file> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( cube_list_file.empty() || cnet_file.empty() ) {
    vw_out() << "ERROR! Require input camera list file and control network.\n";
    vw_out() << "\n" << usage.str() << "\n";
    return 1;
  }

  std::vector<CameraInfo> camera_information;
  std::vector<camera::PinholeModel*> camera_ptrs;

  // Opening Camera List
  std::cout << "Loading cameras:\n";
  std::ifstream camlist( cube_list_file.c_str(), std::ifstream::in );
  if ( !camlist.is_open() )
    vw_throw( ArgumentErr() << "Unable to open \"" << cube_list_file << "\"." );
  std::string cam_buffer;
  std::getline( camlist, cam_buffer );
  while ( !camlist.eof() ) {
    std::cout << "\t\"" << cam_buffer << "\"\n";

    camera::PinholeModel pin;
    camera::CameraModel* isis_model;
    if ( fs::extension( cam_buffer ) == ".isis_adjust" ) {
      typedef boost::shared_ptr<asp::BaseEquation> EqnPtr;
      std::ifstream input( cam_buffer.c_str() );
      EqnPtr posF  = asp::read_equation(input);
      EqnPtr poseF = asp::read_equation(input);
      input.close();
      camera::IsisAdjustCameraModel* camera =
        new camera::IsisAdjustCameraModel( fs::change_extension( cam_buffer,
                                                                 ".cub" ).string(),
                                           posF, poseF );
      isis_model = camera;
      pin =
        linearize_pinhole(camera,
                          Vector2i(camera->samples(),
                                   camera->lines()) );
    } else {
      // Load up camera model and convert
      camera::IsisCameraModel* camera =
        new camera::IsisCameraModel( cam_buffer );
      isis_model = camera;
      pin =
        linearize_pinhole(camera,
                          Vector2i(camera->samples(),
                                   camera->lines()) );
    }

    // Enforcing that the focal lengths are equal (NVM requires it)
    Vector2 focal = pin.focal_length();
    pin.set_focal_length( Vector2(focal[0],focal[0]) );

    // Save and record
    camera_information.push_back( CameraInfo( cam_buffer, isis_model,
                                              pin ) );

    std::getline( camlist, cam_buffer );
  }
  camlist.close();

  // Pull out the pointers
  camera_ptrs.resize( camera_information.size() );
  for( size_t i = 0; i < camera_information.size(); i++ )
    camera_ptrs[i] = &camera_information[i].model;

  // Opening Control Network
  ba::ControlNetwork cnet("nvm");
  cnet.read_binary( cnet_file );

  // Converting control network to simplified control network
  BOOST_FOREACH( ba::ControlPoint& cp, cnet ) {
    BOOST_FOREACH( ba::ControlMeasure& cm, cp ) {
      CameraInfo* cam = &camera_information[cm.image_id()];
      Vector2 ipx = cm.position();
      cm.set_position( cam->model.point_to_pixel(cam->isis_model->camera_center(ipx) +
                                                 cam->isis_model->pixel_to_vector(ipx)) -
                       cam->model.point_offset() );
    }
  }

  // Writing NVM
  write_nvm_iterator_ptr( cnet_file,
                          camera_ptrs.begin(), camera_ptrs.end(),
                          cnet );
}
