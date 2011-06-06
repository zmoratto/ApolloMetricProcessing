#include <vw/Core.h>
#include <vw/FileIO.h>
#include <vw/Image.h>

#include <asp/IsisIO.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/convenience.hpp>

#include "camera_fitting.h"

using namespace vw;
using namespace camera;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Interface Code
int main( int argc, char* argv[] ) {
  std::vector<std::string> input_file_names;
  po::options_description general_options("Options");
  general_options.add_options()
    ("cahv",    "Produce CAHV models and linearize image")
    ("pinhole", "Produce standard projective matrix camera models")
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

  BOOST_FOREACH( std::string const& input, input_file_names ) {
    IsisCameraModel isis_model( input );
    vw_out() << "Loaded " << input << " :\n" << isis_model << "\n";
    std::string serial = isis_model.serial_number();

    const double pixel_scalar = 22900/isis_model.lines();
    const double PRINCIPAL_POINT_MM = (11450.5-1-pixel_scalar*0.5)/200;

    if ( vm.count("cahv") ) {
      CAHVModel cahv = linearize_camera(&isis_model,
                                        Vector2i(isis_model.samples(),
                                                 isis_model.lines()) );
      DiskImageView<int16> input_image( input );
      write_image(fs::change_extension(input,".lin.tif").string(),
                  camera_transform(normalize(input_image,-32767,32767,0,32767), isis_model, cahv,
                                   ZeroEdgeExtension(), BilinearInterpolation()));
      cahv.write( fs::change_extension(input,".cahv").string() );
      continue;
    } else if ( vm.count("pinhole") ) {
      PinholeModel pin = linearize_pinhole(&isis_model,
                                           Vector2i(isis_model.samples(),
                                                    isis_model.lines()) );
      DiskImageView<int16> input_image( input );
      write_image(fs::change_extension(input,".lin.tif").string(),
                  camera_transform(normalize(input_image,-32767,32767,0,32767), isis_model, pin,
                                   ZeroEdgeExtension(), BilinearInterpolation()));
      pin.write( fs::change_extension(input,".pinhole").string() );
      continue;
    }

    PinholeModel pin;
    if ( boost::contains(serial,"APOLLO15") ) {
      vw_out() << "\tApollo 15 Image\n";

        pin = PinholeModel( isis_model.camera_center(),
                            isis_model.camera_pose().rotation_matrix(),
                            76.054, 76.054, PRINCIPAL_POINT_MM, PRINCIPAL_POINT_MM,
                            BrownConradyDistortion(Vector2(-0.006,-0.002),
                                                   Vector3(-.13361854e-5,
                                                           0.52261757e-9,
                                                           -0.50728336e-13),
                                                   Vector2(-.54958195e-6,
                                                           -.46089420e-10),
                                                   2.9659070) );

    } else if ( boost::contains(serial,"APOLLO16") ) {
      vw_out() << "\tApollo 16 Image\n";

        pin = PinholeModel( isis_model.camera_center(),
                            isis_model.camera_pose().rotation_matrix(),
                            75.908, 75.908, PRINCIPAL_POINT_MM, PRINCIPAL_POINT_MM,
                            BrownConradyDistortion(Vector2(-0.010,-0.004),
                                                   Vector3(-0.13678194e-5,
                                                           0.53824020e-9,
                                                           -0.52793282e-13),
                                                   Vector2(0.12275363e-5,
                                                           -0.24596243e-9),
                                                   1.8859721) );

    } else if ( boost::contains(serial,"APOLLO17") ) {
      vw_out() << "\tApollo 17 Image\n";

        pin = PinholeModel( isis_model.camera_center(),
                            isis_model.camera_pose().rotation_matrix(),
                            75.8069, 75.8069, PRINCIPAL_POINT_MM, PRINCIPAL_POINT_MM,
                            BrownConradyDistortion(Vector2(0.0074, 0.0094),
                                                   Vector3(-0.1278842e-5,
                                                           0.5264148e-9,
                                                           -0.5259516e-13),
                                                   Vector2(0.3821279e-6,
                                                           0.1168324e-19),
                                                   3.371325) );
    }
    pin.set_pixel_pitch(pixel_scalar*0.005);
    pin.set_coordinate_frame( Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1) );
    pin.write( fs::change_extension(input,".pinhole").string() );

    { // Also write out a file that has this object's time
      std::ofstream time( fs::change_extension(input,".time").string().c_str() );
      time << input << ", " << std::setprecision(20) << isis_model.ephemeris_time(Vector2()) << std::endl;
      time.close();
    }

    // Test with a wrapping from
    AdjustedCameraModel adjusted( boost::shared_ptr<CameraModel>( new PinholeModel(pin) ), Vector3(), Quaternion<double>(1,0,0,0) );
    boost::shared_ptr<asp::BaseEquation> blank( new asp::PolyEquation(0) );
    IsisAdjustCameraModel isis_adjusted( input, blank, blank );

    Vector3 inworld_frame =pin.pixel_to_vector(Vector2(isis_model.samples(),
                                                       isis_model.lines())/2);

    /*
    std::cout << "PinholeModel   : " << quaternion_to_euler_xyz(pin.camera_pose(Vector2())) << "\n";
    std::cout << "\t" << acos(dot_prod(inverse(pin.camera_pose(Vector2())).rotate(inworld_frame),Vector3(0,0,1))) << "\n";
    std::cout << "AdjustedCamera : " << quaternion_to_euler_xyz(adjusted.camera_pose(Vector2())) << "\n";
    std::cout << "IsisCameraModel: " << quaternion_to_euler_xyz(isis_model.camera_pose(Vector2())) << "\n";
    std::cout << "\t" << acos(dot_prod(inverse(isis_model.camera_pose(Vector2())).rotate(inworld_frame),Vector3(0,0,1))) << "\n";
    std::cout << "IsisAdjustedCam: " << quaternion_to_euler_xyz(isis_adjusted.camera_pose(Vector2())) << "\n";
    */

    try {
      // Perform a test
      double error_sum = 0;
      double count = 0;
      int inc_amt = isis_model.samples()/20;
      for ( int i = 0; i < isis_model.samples(); i += inc_amt ) {
        for ( int j = 0; j < isis_model.lines(); j += inc_amt ) {
          Vector2 starting( i,j );
          Vector3 point = isis_model.pixel_to_vector(starting);
          point = 5e3*point + isis_model.camera_center();
          Vector2 result = pin.point_to_pixel( point );
          Vector2 result_p  = adjusted.point_to_pixel( point );

          error_sum += norm_2(starting-result);
          count++;
          if ( !pin.projection_valid( point ) )
            vw_out() << "\tPROJECTION INVALID\n";
        }
      }
      vw_out() << "\tAverage Pixel Error: "
                << error_sum / count << "\n";
    } catch ( ... ) {
      vw_out() << "FAILED TO PERFORM PROJECTION\n";
      continue;
    }

  }
}
