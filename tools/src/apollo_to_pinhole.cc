#include <vw/Core.h>
#include <vw/Camera/PinholeModel.h>
#include <boost/foreach.hpp>
#include <asp/IsisIO.h>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/convenience.hpp>
#include <vw/Math/EulerAngles.h>

using namespace vw;
using namespace camera;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Interface Code
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

  BOOST_FOREACH( std::string const& input, input_file_names ) {
    IsisCameraModel isis_model( input );
    vw_out() << "Loaded " << input << " :\n" << isis_model << "\n";
    std::string serial = isis_model.serial_number();

    double pixel_scalar = 22900/isis_model.lines();
    double PRINCIPAL_POINT_MM = (11450.5-1-pixel_scalar*0.5)/200;

    PinholeModel pin;
    if ( boost::contains(serial,"APOLLO15") ) {
      vw_out() << "\tApollo 15 Image\n";

      /* This is from a fitting adjustable TSAI
      double distort[6] = {0.007646500298509824,-0.01743067138801845,0.00980946292640812,-2.98092556225311e-05,-1.339089765674149e-05,-1.221974557659228e-05};
      VectorProxy<double,6> distort_v(distort);
      pin = PinholeModel( isis_model.camera_center(),
                          isis_model.camera_pose().rotation_matrix(),
                          3802.7,3802.7,2861.573652036419,2861.768161103493,
                          AdjustableTsaiLensDistortion(distort_v) );
      pin.set_coordinate_frame( pin.coordinate_frame_u_direction(),
                                -pin.coordinate_frame_v_direction(),
                                pin.coordinate_frame_w_direction() );
      */
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
      /*
      double distort[6] = {0.007872316259470486,-0.01786199111078625,0.01016057676230708,5.272930530615576e-06,1.033098038016087e-05,-5.993217765157361e-06};
      VectorProxy<double,6> distort_v(distort);
      pin = PinholeModel( isis_model.camera_center(),
                          isis_model.camera_pose().rotation_matrix(),
                          3796.8,3796.8,2861.397448790403,2861.681737683683,
                          AdjustableTsaiLensDistortion(distort_v) );
      pin.set_coordinate_frame( pin.coordinate_frame_u_direction(),
                                -pin.coordinate_frame_v_direction(),
                                pin.coordinate_frame_w_direction() );
      */
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
