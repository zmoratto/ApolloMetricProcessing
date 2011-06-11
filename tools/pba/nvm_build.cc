// Boost
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Vision Workbench
#include <vw/Math.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
using namespace vw;

// Stereo Pipeline
#include <asp/Core/Common.h>
#include "../pba/nvmio.h"

struct Options : public asp::BaseOptions {
  std::vector<std::string> input_names;
  std::vector<Matrix3x3f>  rotations;
  std::vector<Vector3f>    translations;
  std::vector<float>       focal_lengths;
  int32 min_matches;

  std::string nvm_output;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("o,output-nvm", po::value(&opt.nvm_output)->default_value("built"),
     "Output name for NVM file.")
    ("min-matches", po::value(&opt.min_matches)->default_value(5));
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.input_names));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <singleton nvm> ...\n";

  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             positional, positional_desc, usage.str() );

  if ( opt.input_names.empty() )
    vw_throw( ArgumentErr() << "Missing input nvm files!\n"
              << usage.str() << general_options );
}

int main( int argc, char* argv[] ) {
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Reading in NVM
    opt.rotations.resize( opt.input_names.size() );
    opt.translations.resize( opt.input_names.size() );
    opt.focal_lengths.resize( opt.input_names.size() );
    size_t index = 0;
    BOOST_FOREACH( std::string const& input, opt.input_names ) {
      std::string key;
      size_t num_cameras;
      std::ifstream file( input.c_str(), std::ios::in );
      if (!file.is_open() )
        vw_throw( ArgumentErr() << "Unable to open: " << input << "!\n" );
      file >> key >> num_cameras;
      VW_ASSERT( num_cameras == 1, ArgumentErr() << "Input NVM is supposed to only contain a single camera!\n" );

      std::string name;
      int buf;
      file >> name >> opt.focal_lengths[index]
           >> opt.rotations[index](0,0) >> opt.rotations[index](0,1) >> opt.rotations[index](0,2)
           >> opt.rotations[index](1,0) >> opt.rotations[index](1,1) >> opt.rotations[index](1,2)
           >> opt.rotations[index](2,0) >> opt.rotations[index](2,1) >> opt.rotations[index](2,2)
           >> opt.translations[index][0] >> opt.translations[index][1] >> opt.translations[index][2]
           >> buf >> buf;
      index++;
    }

    // Construct Pinhole Camera Models
    std::vector< boost::shared_ptr<camera::CameraModel> > camera_models;
    {
      TerminalProgressCallback tpc("","Building Models:");
      tpc.report_progress(0);
      float inc_amt = 1.0/float(opt.input_names.size());
      for ( size_t i = 0; i < opt.input_names.size(); i++ ) {
        tpc.report_incremental_progress(inc_amt);
        Vector3f camera_center =
          -transpose(opt.rotations[i])*opt.translations[i];
        camera_models.push_back( boost::shared_ptr<camera::CameraModel>(
          new camera::PinholeModel( camera_center, transpose(opt.rotations[i]),
                                    opt.focal_lengths[i], opt.focal_lengths[i], 0, 0,
                                    Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1),
                                    camera::NullLensDistortion() ) ) );
      }
      tpc.report_finished();
    }

    // Build Control Network
    ba::ControlNetwork cnet( "Rawr" );
    build_control_network( cnet, camera_models,
                           opt.input_names, opt.min_matches );

    // Rewrite as NVM
    write_nvm_r9t( opt.nvm_output,
                   opt.focal_lengths.begin(), opt.focal_lengths.end(),
                   opt.rotations.begin(), opt.rotations.end(),
                   opt.translations.begin(), opt.translations.end(),
                   cnet );
  } catch( Exception const& e) {
    std::cerr << "\n\nVW Error: " << e.what() << std::endl;
    return 1;
  } catch( std::exception const& e ) {
    std::cerr << "\n\nVW Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
