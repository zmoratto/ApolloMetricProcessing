#include <vw/Core.h>
#include <asp/IsisIO.h>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/convenience.hpp>

#include "../pba/nvmio.h"

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main( int argc, char* argv[] ) {
  std::string cube_list_file, nvm_file;
  po::options_description general_options("Options");
  general_options.add_options()
    ("cube-list", po::value(&cube_list_file), "A file listing the input cube files.")
    ("nvm-file", po::value(&nvm_file), "Input control network.")
    ("sanity-check", "Debug, run sanity check" )
    ("help,h", "Display this help message");

  po::positional_options_description p;
  p.add("cube-list", 1);
  p.add("nvm-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(general_options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <list-file> <nvm-file> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( cube_list_file.empty() || nvm_file.empty() ) {
    vw_out() << "ERROR! Require input camera list file and control network.\n";
    vw_out() << "\n" << usage.str() << "\n";
    return 1;
  }

  // Opening Camera List
  std::cout << "Opening \"" << cube_list_file << "\"\n";
  std::ifstream camlist( cube_list_file.c_str(), std::ifstream::in );
  if ( !camlist.is_open() )
    vw_throw( ArgumentErr() << "Unable to open camlist!" );
  std::string camlist_buffer;
  std::getline( camlist, camlist_buffer );

  // Opening NVM file
  std::cout << "Opening \"" << nvm_file << "\"\n";
  std::ifstream nvm( nvm_file.c_str(), std::ifstream::in );
  if ( !nvm.is_open() )
    vw_throw( ArgumentErr() << "Unable to open NVM file!" );
  size_t nvm_num_cameras;
  {
    std::string key;
    nvm >> key >> nvm_num_cameras;
  }

  // Generating isis_adjust files
  size_t count = 0;
  while ( !camlist.eof() ) {
    std::cout << "\t\"" << camlist_buffer << "\"\n";

    count++;
    if ( count > nvm_num_cameras )
      vw_throw( ArgumentErr() << "Number of cameras mismatch between NVM and Camlist!\n" );

    camera::IsisCameraModel org_model( camlist_buffer );
    boost::scoped_ptr<camera::CameraModel> fix_model( read_nvm_camera( nvm ) );

    // Working out the difference between the cameras!
    Vector3 pos_correction =
      fix_model->camera_center(Vector2()) - org_model.camera_center(Vector2());
    Quat pose_correction =
      org_model.camera_pose(Vector2()) * inverse(fix_model->camera_pose(Vector2()));
    Vector3 posev_correction = pose_correction.axis_angle();

    // Writing the adjust file
    typedef boost::shared_ptr<asp::BaseEquation> EqnPtr;
    EqnPtr posF( new asp::PolyEquation( 0 ) );
    (*posF)[0] = pos_correction[0];
    (*posF)[1] = pos_correction[1];
    (*posF)[2] = pos_correction[2];
    EqnPtr poseF( new asp::PolyEquation( 0 ) );
    (*poseF)[0] = posev_correction[0];
    (*poseF)[1] = posev_correction[1];
    (*poseF)[2] = posev_correction[2];
    std::ofstream adjust( fs::change_extension( camlist_buffer, ".isis_adjust" ).string().c_str(),
                          std::ofstream::out );
    asp::write_equation( adjust, posF );
    asp::write_equation( adjust, poseF );

    if ( vm.count("sanity-check") ) {
      camera::IsisAdjustCameraModel san_model( camlist_buffer, posF, poseF );
      std::cout << fix_model->camera_center(Vector2()) << " " << san_model.camera_center(Vector2()) << "\n";
      std::cout << fix_model->camera_pose(Vector2()) <<   " " << san_model.camera_pose(Vector2()) << "\n";
    }
  }
}
