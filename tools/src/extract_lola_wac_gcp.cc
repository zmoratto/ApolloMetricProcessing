#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include <asp/IsisIO/IsisAdjustCameraModel.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;
#include <boost/foreach.hpp>

using namespace vw;
using namespace vw::camera;

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
  usage << "Usage: " << argv[0] << "[options] <image-files> ...\n\n";
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( input_file_names.size() == 0 ) {
    vw_out() << "ERROR! Require an input file.\n";
    vw_out() << "\n" << usage.str() << "\n";
    return 1;
  }

  // CONSTANTS for maps. Sorry guys this is specific for my system.
  // The WAC mosaic is 20GB and is currently private.
  std::string lola_file("/Users/zmoratto/Data/Moon/LOLA/LOLA_64px_p_deg_DEM.tif");
  std::string wac_file("/Users/zmoratto/Data/Moon/LROWAC/global_100m_JanFeb_and_JulyAug.180.cub");
  cartography::GeoReference lola_georef, wac_georef;
  cartography::read_georeference( lola_georef, lola_file );
  cartography::read_georeference( wac_georef,  wac_file );


  // Create Control Network
  // .. TODO ..

  // Generating Data
  BOOST_FOREACH( std::string const& camera_file, input_file_names ) {
    // Create Camera Model
    boost::shared_ptr<CameraModel> model;
    std::string adjust_file =
      fs::path( camera_file ).replace_extension("isis_adjust").string();
    if ( fs::exists( adjust_file ) ) {
      vw_out() << "Loading \"" << adjust_file << "\"\n";
      std::ifstream input( adjust_file.c_str() );
      boost::shared_ptr<asp::BaseEquation> posF = asp::read_equation(input);
      boost::shared_ptr<asp::BaseEquation> poseF = asp::read_equation(input);
      input.close();
      model = boost::shared_ptr<CameraModel>(new IsisAdjustCameraModel( camera_file, posF, poseF ) );
    } else {
      vw_out() << "Loading \"" << camera_file << "\"\n";
      model = boost::shared_ptr<CameraModel>(new IsisCameraModel( camera_file ) );
    }

    // Extract serials
    std::string serial;
    Vector2i size;
    if ( fs::exists( adjust_file ) ) {
      boost::shared_ptr<IsisAdjustCameraModel> cam =
        boost::shared_dynamic_cast< IsisAdjustCameraModel >(model);
      serial = cam->serial_number();
      size = Vector2i(cam->samples(),cam->lines());
    } else {
      boost::shared_ptr<IsisCameraModel> cam =
        boost::shared_dynamic_cast< IsisCameraModel >(model);
      serial = cam->serial_number();
      size = Vector2i(cam->samples(),cam->lines());
    }

    // Working out scale and rotation
    float degree_scale;
    BBox2 degree_bbox =
      cartography::camera_bbox( lola_georef, model, size[0], size[1], degree_scale );
    degree_bbox.expand(2);
    bool working;
    Vector2 l_direction =
      cartography::geospatial_intersect( Vector2(), lola_georef, model,
                                         1, working ) -
      cartography::geospatial_intersect( Vector2(0,size[1]), lola_georef, model,
                                         1, working );
    std::cout << "L direction: " << l_direction << "\n";
    Vector2 r_direction =
      cartography::geospatial_intersect( Vector2(size[0],0), lola_georef, model,
                                         1, working ) -
      cartography::geospatial_intersect( size, lola_georef, model,
                                         1, working );
    std::cout << "R direction: " << r_direction << "\n";
    double rotate = M_PI/2 - (atan2(l_direction[1],l_direction[0]) +
                     atan2(r_direction[1],r_direction[0]) )/2;
    degree_scale = (norm_2(l_direction)+norm_2(r_direction)) / (size[1]*2);

    // Debug information
    vw_out() << "BBox: " << degree_bbox << "\n";
    vw_out() << "Scle: " << degree_scale << "\n"; // Degrees / pixel I assume
    vw_out() << "Rot : " << rotate*180/M_PI << "\n\n";

    // Creating output georef
    cartography::GeoReference georef_out;
    georef_out.set_datum( cartography::Datum("Sphere","Sphere","Zero",
                                             180.0/M_PI, 180.0/M_PI, 0) );
    georef_out.set_orthographic( degree_bbox.min()[1] + degree_bbox.height()/2,
                                 degree_bbox.min()[0] + degree_bbox.width()/2 );
    // Transform defines pixel to point
    Matrix<double> tx = math::identity_matrix<3>();
    tx(0,2) = -size[0]*degree_scale/2;
    tx(1,2) = size[1]*degree_scale/2;
    tx(0,0) = degree_scale*1.1;
    tx(1,1) = -degree_scale*1.1;
    georef_out.set_transform( tx );
    std::cout << "Georef: " << georef_out << "\n";

    std::cout << "Size: " << size/2 << "\n";
    std::cout << "Size: " << size << "\n";

    // Rastering an image to perform transform

    // crops more of the image than we want
    cartography::GeoTransform geotx( wac_georef, georef_out );
    DiskImageView<PixelGray<uint8> > input( wac_file );
    ImageViewRef<PixelGray<uint8> > output1 =
      crop( transform( input,
                       compose( RotateTransform( -rotate, (Vector2(size)-Vector2(1,1))/2 ),
                                geotx
                                ) ),
            BBox2i(0,0,size[0],size[1]) );
    write_image( fs::path(camera_file).replace_extension(".wac.tif").string(),
                 normalize(output1) );
  }
}
