#include <vw/Core.h>
#include <vw/BundleAdjustment.h>
#include <asp/IsisIO.h>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/convenience.hpp>

#include "camera_fitting.h"

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct CameraInfo {
  CameraInfo( std::string const& f,
              camera::IsisCameraModel const& i,
              camera::PinholeModel const& p ) :
    filename(f), isis_model(i), model(p) {}
  std::string filename;
  camera::IsisCameraModel isis_model;
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

  // Opening Camera List
  std::cout << "Loading cameras:\n";
  std::ifstream camlist( cube_list_file.c_str(), std::ifstream::in );
  if ( !camlist.is_open() )
    vw_throw( ArgumentErr() << "Unable to open \"" << cube_list_file << "\"." );
  std::string cam_buffer;
  std::getline( camlist, cam_buffer );
  while ( !camlist.eof() ) {
    std::cout << "\t\"" << cam_buffer << "\"\n";

    // Load up camera model and convert
    camera::IsisCameraModel isis_model( cam_buffer );
    camera::PinholeModel pin =
      linearize_pinhole(&isis_model,
                        Vector2i(isis_model.samples(),
                                 isis_model.lines()) );

    // Enforcing that the focal lengths are equal (NVM requires it)
    Vector2 focal = pin.focal_length();
    pin.set_focal_length( Vector2(focal[0],focal[0]) );

    // Save and record
    camera_information.push_back( CameraInfo( cam_buffer, isis_model,
                                              pin ) );

    std::getline( camlist, cam_buffer );
  }
  camlist.close();

  // Opening Control Network
  ba::ControlNetwork cnet("nvm");
  cnet.read_binary( cnet_file );

  // Converting control network to simplified control network
  BOOST_FOREACH( ba::ControlPoint& cp, cnet ) {
    BOOST_FOREACH( ba::ControlMeasure& cm, cp ) {
      CameraInfo* cam = &camera_information[cm.image_id()];
      Vector2 ipx = cm.position();
      cm.set_position( cam->model.point_to_pixel(cam->isis_model.camera_center(ipx) +
                                                 cam->isis_model.pixel_to_vector(ipx)) -
                       cam->model.point_offset() );
    }
  }

  // Writing NVM
  std::ofstream nvm( fs::change_extension( cnet_file, ".nvm").string().c_str(),
                     std::ofstream::out );
  nvm << std::setprecision(12);
  // Writing camera models
  nvm << "NVM_V3_R9T\n";
  nvm << camera_information.size() << "\n";
  BOOST_FOREACH( CameraInfo const& camera, camera_information ) {
    nvm << camera.filename << " " << camera.model.focal_length()[0] << " ";
    Matrix3x3 rot = transpose(camera.model.camera_pose().rotation_matrix());
    nvm << rot(0,0) << " " << rot(0,1) << " " << rot(0,2) << " "
        << rot(1,0) << " " << rot(1,1) << " " << rot(1,2) << " "
        << rot(2,0) << " " << rot(2,1) << " " << rot(2,2) << " ";
    Vector3 ctr = rot * (-camera.model.camera_center());
    nvm << ctr[0] << " " << ctr[1] << " " << ctr[2] << " 0 0\n";
  }
  // Writing point measurements
  nvm << cnet.size() - cnet.num_ground_control_points() << "\n";
  BOOST_FOREACH( ba::ControlPoint const& cp, cnet ) {
    if ( cp.type() == ba::ControlPoint::GroundControlPoint )
      continue;
    nvm << cp.position()[0] << " " << cp.position()[1] << " "
        << cp.position()[2] << " 0 0 0 " << cp.size();
    BOOST_FOREACH( ba::ControlMeasure const& cm, cp ) {
      nvm << " " << cm.image_id() << " 0 "
          << cm.position()[0] << " " << cm.position()[1];
    }
    nvm << "\n";
  }
  nvm.close();
}
