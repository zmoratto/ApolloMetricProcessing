// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/InterestPoint.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::ip;
using namespace vw::ba;

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
namespace po = boost::program_options;

#include <asp/IsisIO/IsisCameraModel.h>
#include <iostream>
#include <fstream>

int main( int argc, char** argv) {
  std::string input_cnet, input_list;

  po::options_description general_options("Options");
  general_options.add_options()
    ("list,l", po::value<std::string>(&input_list), "List of cubes")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-file", po::value<std::string>(&input_cnet));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-file", -1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <filenames>..." << std::endl << std::endl;
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch (po::error &e ) {
    std::cout << "An error occured while parsing command line arguments.\n";
    std::cout << "\t" << e.what() << "\n\n";
    std::cout << usage.str();
    return 1;
  }

  if( vm.count("help") ) {
    vw_out() << usage.str();
    return 1;
  }

  if( input_cnet == "" ) {
    vw_out() << "Input control network not provided.\n";
    return 1;
  }

  // Opening existing
  std::vector<std::string> tokens;
  boost::split( tokens, input_cnet, boost::is_any_of(".") );
  boost::shared_ptr<ControlNetwork> cnet = boost::shared_ptr<ControlNetwork>( new ControlNetwork("USGS-ified Control Network"));
  if ( tokens[tokens.size()-1] == "net" ) {
    vw_out() << "Reading ISIS Control Network: " << input_cnet << " ";
    cnet->read_isis( input_cnet );
  } else if ( tokens[tokens.size()-1] == "cnet" ) {
    vw_out() << "Reading VW Control Network: " << input_cnet << " ";
    cnet->read_binary( input_cnet );
  } else {
    vw_throw( IOErr() << "Unknown Control Network file extension, \""
              << tokens[tokens.size()-1] << "\"." );
  }
  vw_out() << " .. Fin.\n";

  // Loading up List of Camera and getting serial numbers
  std::map<std::string,std::string> serial_to_name;
  {
    std::ifstream list_of_cubes(input_list.c_str());
    std::string buf;
    while ( !list_of_cubes.eof() ) {
      std::getline(list_of_cubes,buf);
      if ( buf != "" ) {
        IsisCameraModel cam(buf);
        int index = buf.rfind("/");
        index++;
        std::string name = buf.substr(index,buf.size()-index);
        std::cout << "Reading: " << name << "\n";
        serial_to_name[cam.serial_number()]=name;
      }
    }
    list_of_cubes.close();
  }

  // Perform editing here
  vw_out() << "Converting XYZ to LLA\n";
  double inc_amt = 1.0/double(cnet->size());
  TerminalProgressCallback tpc;
  for ( ControlNetwork::iterator pt = cnet->begin();
        pt != cnet->end(); pt++ ) {
    tpc.report_incremental_progress(inc_amt);
    Vector3 position = pt->position();
    Vector3 lla = cartography::xyz_to_lon_lat_radius( position );
    pt->set_position(lla);
    for ( ControlPoint::iterator m = pt->begin();
          m != pt->end(); m++ ) {
      std::string serial = m->serial();
      std::string name = serial_to_name[serial];
      m->set_serial( name );
      Vector2 px = m->position();
      px += Vector2(1,1);
      px *= 4;
      m->set_position( px );
    }
  }
  tpc.report_finished();

  // Writing usgs
  vw_out() << "Writing out USGS-ified control network\n";
  cnet->write_isis("usgs_"+tokens[0]);

  return 0;
}
