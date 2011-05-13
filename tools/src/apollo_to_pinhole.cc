#include <vw/Core.h>
#include <vw/Camera.h>
#include <boost/foreach.hpp>
#include <asp/IsisIO.h>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/convenience.hpp>
#include <vw/Math/EulerAngles.h>
#include <vw/FileIO.h>
#include <vw/Image.h>

using namespace vw;
using namespace camera;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct CAHVOptimizeFunctor : public math::LeastSquaresModelBase<CAHVOptimizeFunctor> {
  typedef Vector<double,18> result_type;
  typedef Vector<double,9>  domain_type;
  typedef Matrix<double>  jacobian_type;

  CameraModel* m_cam;
  const std::vector<Vector2> m_measure;
  CAHVOptimizeFunctor(CameraModel* cam,
                      std::vector<Vector2> const& measure ) : m_cam(cam), m_measure(measure) {}

  inline result_type operator()( domain_type const& x ) const {
    CAHVModel cahv(m_cam->camera_center(Vector2()),
                   subvector(x,0,3), subvector(x,3,3), subvector(x,6,3) );
    cahv.A = normalize(cahv.A);
    result_type output;
    for ( size_t i = 0; i < 9; i++ )
      subvector(output,2*i,2) = m_measure[i] -
        cahv.point_to_pixel(cahv.C + 1000*m_cam->pixel_to_vector(m_measure[i]) );
    return output;
  }
};

struct PinholeOptimizeFunctor : public math::LeastSquaresModelBase<PinholeOptimizeFunctor> {
  typedef Vector<double, 18> result_type;
  typedef Vector<double, 8>  domain_type;
  typedef Matrix<double>     jacobian_type;

  CameraModel* m_cam;
  const std::vector<Vector2> m_measure;
  PinholeOptimizeFunctor( CameraModel* cam,
                          std::vector<Vector2> const& measure ) : m_cam(cam), m_measure(measure) {}

  inline result_type operator()( domain_type const& x ) const {
    PinholeModel ccam = to_pinhole( x, m_cam->camera_center(Vector2()) );
    result_type output;
    for ( size_t i = 0; i < 9; i++ )
      subvector(output,2*i,2) = m_measure[i] -
        ccam.point_to_pixel(m_cam->camera_center(Vector2()) +
                            1000*m_cam->pixel_to_vector(m_measure[i]));
    return output;
  }

  static domain_type to_vec( PinholeModel const& model ) {
    domain_type vec;
    Quat pose = model.camera_pose(Vector2());
    for ( size_t i = 0; i < 4; i++ )
      vec[i] = pose[i];
    subvector( vec, 4, 2 ) = model.focal_length();
    subvector( vec, 6, 2 ) = model.point_offset();
    return vec;
  }

  static PinholeModel to_pinhole( domain_type const& vec, Vector3 const& center ) {
    return PinholeModel( center,
                         Quat( subvector(vec,0,4) ).rotation_matrix(),
                         vec[4], vec[5], vec[6], vec[7],
                         Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1),
                         NullLensDistortion() );
  }
};

PinholeModel linearize_pinhole( CameraModel* cam, Vector2i const& size ) {
  Vector2 center_px  = (size - Vector2(1,1))/2.0;
  Vector3 center_vec = cam->pixel_to_vector( center_px );
  Vector3 off_x_vec  = cam->pixel_to_vector( center_px + Vector2i(10,0) );
  Vector3 off_y_vec  = cam->pixel_to_vector( center_px + Vector2i(0,10) );

  double fu = 10.0 / tan( acos( dot_prod(off_x_vec, center_vec ) ) );
  double fv = 10.0 / tan( acos( dot_prod(off_y_vec, center_vec ) ) );

  PinholeModel initial( cam->camera_center(center_px), cam->camera_pose(center_px).rotation_matrix(),
                        fu, fv, -size[0]/2, -size[1]/2 );
  initial.set_coordinate_frame( Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1) );

  std::vector<Vector2> input(9);
  input[0] = Vector2();
  input[1] = Vector2(size[0]/2,0);
  input[2] = Vector2(size[0]-1,0);
  input[3] = Vector2(size[0]-1,size[1]/2);
  input[4] = Vector2(size[0]-1,size[1]-1);
  input[5] = Vector2(size[0]/2,size[1]-1);
  input[6] = Vector2(0        ,size[1]-1);
  input[7] = Vector2(0        ,size[1]/2);
  input[8] = center_px;

  int status;
  Vector<double> seed = PinholeOptimizeFunctor::to_vec( initial );
  Vector<double> sol =
    math::levenberg_marquardt( PinholeOptimizeFunctor( cam, input ),
                               seed, Vector<double,18>(), status );
  std::cout << "Status  : " << status << "\n";
  std::cout << "Error   : " << norm_2( PinholeOptimizeFunctor(cam,input)(sol) ) << "\n";
  return PinholeOptimizeFunctor::to_pinhole( sol, cam->camera_center(center_px) );
}

CAHVModel linearize_camera( CameraModel* cam, Vector2i const& size ) {
  CAHVModel output;
  Vector2 center_px = (size - Vector2(1,1))/2.0;
  output.C = cam->camera_center(Vector2());
  output.A = cam->pixel_to_vector( center_px );
  output.H =
    normalize(cross_prod(cam->pixel_to_vector(center_px+Vector2(0,1)),
                         cam->pixel_to_vector(center_px)));
  output.V =
    normalize(cross_prod(cam->pixel_to_vector(center_px),
                         cam->pixel_to_vector(center_px+Vector2(1,0))));

  double h_angle =
    M_PI/2 - acos(dot_prod(cam->pixel_to_vector(Vector2(size[0]-1,center_px[1])),
                           output.H));
  double h_focal = size[0]*0.5/tan(h_angle);

  double v_angle =
    M_PI/2 - acos(dot_prod(cam->pixel_to_vector(Vector2(center_px[0],size[1]-1)),
                           output.V));
  double v_focal = size[1]*0.5/tan(v_angle);


  output.H = Vector3(-(size[0]-1)/2,0,h_focal);
  output.V = Vector3(-(size[1]-1)/2,0,v_focal);

  std::vector<Vector2> input(9);
  input[0] = Vector2();
  input[1] = Vector2(size[0]/2,0);
  input[2] = Vector2(size[0]-1,0);
  input[3] = Vector2(size[0]-1,size[1]/2);
  input[4] = Vector2(size[0]-1,size[1]-1);
  input[5] = Vector2(size[0]/2,size[1]-1);
  input[6] = Vector2(0        ,size[1]-1);
  input[7] = Vector2(0        ,size[1]/2);
  input[8] = center_px;

  Vector<double,9> seed;
  subvector(seed,0,3) = output.A;
  subvector(seed,3,3) = output.H;
  subvector(seed,6,3) = output.V;
  std::cout << "Seed: " << seed << "\n";

  int status;
  Vector<double> camera =
    math::levenberg_marquardt( CAHVOptimizeFunctor( cam, input ),
                               seed, Vector<double,18>(), status );
  output.A = normalize(subvector(camera,0,3));
  output.H = subvector(camera,3,3);
  output.V = subvector(camera,6,3);
  std::cout << "Status  : " << status << "\n";
  std::cout << "Error   : " << norm_2( CAHVOptimizeFunctor(cam,input)(camera) ) << "\n";

  return output;
}

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
