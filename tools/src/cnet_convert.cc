// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::ba;
using namespace vw::camera;

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem/path.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Core/Macros.h>

#include <fstream>

struct Options {
  // Input
  std::string cnet_file;
  std::string camera_list_file;

  bool convert_isis, convert_serial_to_name;

  // Output
  std::string output_prefix;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("convert-isis", "Convert ISIS style cnet to VW style.")
    ("convert-serial-to-name", "Convert the serial numbers in cnet to filenames.")
    ("output-prefix,o", po::value(&opt.output_prefix)->default_value("converted"), "Output prefix for new control network.")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("cnet-file", po::value(&opt.cnet_file) )
    ("camera-list-file", po::value(&opt.camera_list_file) );

  po::positional_options_description positional_desc;
  positional_desc.add("cnet-file", 1 );
  positional_desc.add("camera-list-file", 1 );

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <cnet_file> <list_of_cameras>\n";

  opt.convert_isis = vm.count("input-files");
  opt.convert_serial_to_name = vm.count("convert-serial-to-name");

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( !vm.count("cnet-file") || !vm.count("camera-list-file") )
    vw_throw( ArgumentErr() << "Missing all of the required input files.\n"
              << usage.str() << general_options );
}

int main( int argc, char** argv) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    ControlNetwork cnet("input");
    if ( opt.convert_isis ) {
      // Convert ISIS style to VW style
      // We expect the input to be control network plus list of cameras
      cnet.read_isis( opt.cnet_file );
    } else {
      // Convert VW style to ISIS
      // We expect the input to be control network plus list of cameras
      cnet.read_binary( opt.cnet_file );
    }

    // Reading in cameras
    std::map<std::string,std::string> serial_to_name;
    std::map<std::string,std::string> name_to_serial;
    if ( opt.convert_serial_to_name ) {
      {
        TerminalProgressCallback tpc( "", "Loading Cameras:" );
        std::ifstream list_of_cubes(opt.camera_list_file.c_str());
        std::string buf;
        int count = 0;
        while ( list_of_cubes.good() ) {
          std::getline(list_of_cubes,buf);
          count++;
        }
        list_of_cubes.close();
        list_of_cubes.open(opt.camera_list_file.c_str());

        float inc_amt = 1.0/float(count);
        std::getline(list_of_cubes,buf);
        while ( list_of_cubes.good() ) {
          tpc.report_incremental_progress(inc_amt);
          if ( buf != "" ) {
            IsisCameraModel cam(buf);
            std::string name = fs::path(buf).filename();
            serial_to_name[cam.serial_number()]=name;
            name_to_serial[name]=cam.serial_number();
          }
          std::getline(list_of_cubes,buf);
        }
        tpc.report_finished();
        list_of_cubes.close();
      }
    }

    if ( opt.convert_isis ) {
      BOOST_FOREACH( ControlPoint& cp, cnet ) {
        Vector3 position = cp.position();
        Vector3 xyz = cartography::lon_lat_radius_to_xyz( position );
        cp.set_position(xyz);
        BOOST_FOREACH( ControlMeasure& cm, cp ) {
          if ( opt.convert_serial_to_name ) {
            std::string serial = cm.serial();
            std::string name = serial_to_name[serial];
            cm.set_serial( name );
          }
          Vector2 px = cm.position();
          px -= Vector2(1,1);
          cm.set_position( px );
        }
      }
      vw_out() << "Writing Vision Workbench control network.\n";
      cnet.write_binary( opt.output_prefix );
    } else {
      BOOST_FOREACH( ControlPoint& cp, cnet ) {
        Vector3 position = cp.position();
        Vector3 lla = cartography::xyz_to_lon_lat_radius( position );
        cp.set_position(lla);
        BOOST_FOREACH( ControlMeasure& cm, cp ) {
          if ( opt.convert_serial_to_name ) {
            std::string serial = cm.serial();
            std::string name = serial_to_name[serial];
            cm.set_serial( name );
          }
          Vector2 px = cm.position();
          px += Vector2(1,1);
          cm.set_position( px );
        }
      }
      vw_out() << "Writing ISIS control network.\n";
      cnet.write_isis( opt.output_prefix );
    }

  } ASP_STANDARD_CATCHES;

}
