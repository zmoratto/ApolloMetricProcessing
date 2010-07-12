#include <vw/Core.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Cartography.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include <asp/IsisIO/IsisCameraModel.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
using namespace vw::ba;

static std::string lola_name = "/Users/zmoratto/Data/Moon/LOLA/LOLA_64px_p_deg_DEM.tif";

int main( int argc, char* argv[] ) {

  std::string input_cnet, latlon_file;
  std::vector<std::string> cube_files;
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-cnet", po::value(&input_cnet))
    ("latlon-file", po::value(&latlon_file))
    ("cube-files",po::value(&cube_files));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-cnet", 1);
  p.add("latlon-file", 1);
  p.add("cube-files", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <isis-cnet> <latlon-file> <cube files .. > ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") || input_cnet.empty() || latlon_file.empty() ||
      cube_files.empty() ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  }

  ControlNetwork cnet("");
  cnet.read_isis( input_cnet );

  // Serial Number to Camera Index
  std::map<std::string,int> serial_to_cam;
  std::map<int,std::string> cam_to_filename;
  {
    int count = 0;
    BOOST_FOREACH( std::string const& str, cube_files ) {
      IsisCameraModel cam(str);
      serial_to_cam[cam.serial_number()] = count;
      cam_to_filename[count] = str;
      count++;
    }
  }
  BOOST_FOREACH( ControlPoint & cp, cnet ) {
    BOOST_FOREACH( ControlMeasure & cm, cp ) {
      cm.set_image_id(serial_to_cam[cm.serial()]);
    }
  }

  // Loading up LOLA
  GeoReference lolaref;
  read_georeference( lolaref, lola_name );
  std::cout << "LOLA GeoRef:\n" << lolaref << "\n\n";
  DiskImageView<float> lola( lola_name );

  std::ifstream ifile(latlon_file.c_str());

  // Processing each CP to a ground control point file
  int count = 1;
  BOOST_FOREACH( ControlPoint const& cp, cnet ) {
    std::cout << "Working Control Point: " << cp << "\n";

    BOOST_FOREACH( ControlMeasure const& cm, cp ) {
      std::cout << cm.serial() << "\n";
    }

    double lat, lon;
    ifile >> lat >> lon;

    Vector2 lola_px = lolaref.lonlat_to_pixel(Vector2(lon,lat));
    double radius = lola(lola_px[0],lola_px[1]) + lolaref.datum().radius(lon,lat);

    std::cout << "Lat: " << lat << " Lon: " << lon
              << " Rad: " << radius << "\n";

    std::ostringstream ostr;
    ostr << "usgs_small_" << count << ".gcp";
    std::ofstream ofile(ostr.str().c_str());

    ofile << lon << " " << lat << " " << radius << " 300 300 300\n";
    BOOST_FOREACH( ControlMeasure const& cm, cp ) {
      ofile << cam_to_filename[cm.image_id()] << " " << cm.position()[0] << " "
            << cm.position()[1] << "\n";
    }
    ofile.close();

    count++;
  }

}
