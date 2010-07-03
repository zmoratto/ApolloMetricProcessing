#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Cartography.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

int main( int argc, char* argv[] ) {
  std::vector<std::string> input_file_names;
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_file_names));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <image-files> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( input_file_names.size() == 0 ) {
    vw_out() << "ERROR! Require an input file.\n";
    vw_out() << "\n" << usage.str() << "\n";
    return 1;
  }

  std::vector<std::string> clementine_names;
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_30n045_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_30n135_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_30n225_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_30n315_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_30s045_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_30s135_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_30s225_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_30s315_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_75n090_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_75n270_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_75s090_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/ClementineBaseMapV2/clembase_75s270_256ppd.tif");

  // Load up those files georeference information
  std::vector<GeoReference> clementine_georef;
  std::vector<BBox2i> clementine_bbox;
  BOOST_FOREACH( std::string const& names, clementine_names ) {
    GeoReference georef;
    read_georeference( georef, names );
    clementine_georef.push_back(georef);
    DiskImageView<PixelGray<uint8> > dimage( names );
    clementine_bbox.push_back(bounding_box(dimage));
  }

  // Load up the camera models
  BOOST_FOREACH( std::string const& pinhole_name, input_file_names ) {
    PinholeModel pinhole( pinhole_name );

    Vector3 llr = XYZtoLonLatRadFunctor()( pinhole.camera_center() );

    // Finding an image that actually contains this camera's location
    for ( unsigned i = 0; i < clementine_names.size(); i++ ) {
      Vector2 clem_pix = clementine_georef[i].point_to_pixel(clementine_georef[i].lonlat_to_point(subvector(llr,0,2)));
      if ( clementine_bbox[i].contains(clem_pix) ) {
        // Create a new GeoReference
        GeoReference georef_dest;
        georef_dest.set_well_known_geogcs("D_MOON");
        georef_dest.set_orthographic( llr[1], llr[0] );
        Matrix<double> itx = vw::math::identity_matrix<3>();
        double zoom = 40;
        itx(0,0) = zoom;
        itx(1,1) = -zoom;
        //itx(0,2) = itx(1,2) = 00*zoom;
        itx(0,2) = -2000*zoom;
        itx(1,2) = 2000*zoom;
        georef_dest.set_transform(itx);
        GeoTransform geotx( clementine_georef[i], georef_dest );

        DiskImageView<PixelGray<uint8> > input(clementine_names[i]);
        ImageViewRef<PixelGray<uint8> > output = crop( transform( input, geotx, ZeroEdgeExtension(), BicubicInterpolation() ), BBox2i(0,0,4000,4000) );

        //std::map<std::string,std::string> gdal_options;
        //        gdal_options["COMPRESS"] = "NONE";
        //DiskImageResourceGDAL rsrc( fs::path(pinhole_name).replace_extension().string()+".clem.jpg", output.format(),
        //                                    Vector2i(256,256), gdal_options );
        //write_georeference( rsrc, georef_dest );
        write_image( fs::path(pinhole_name).replace_extension().string()+".clem.jpg", output, TerminalProgressCallback("test","Writing:"));

        break;
      }
    }
  }
}
