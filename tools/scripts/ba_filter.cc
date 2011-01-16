/// BA Filter
///
/// This will filter interest points by trying to bundle adjust 2
/// cameras together. This only works for images that are cubes.

#include <vw/InterestPoint.h>
#include <vw/BundleAdjustment.h>
#include <asp/IsisIO.h>
#include <asp/Core/Macros.h>
using namespace vw;
using namespace vw::camera;

#include <boost/foreach.hpp>
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;

class FilterBAModel : public ba::ModelBase< FilterBAModel, 6, 3 > {
  typedef Vector<double, 6> camera_vector_t;
  typedef Vector<double, 3> point_vector_t;

  std::vector< boost::shared_ptr<CameraModel> > m_cameras;
  boost::shared_ptr<ba::ControlNetwork> m_cnet;
  std::vector<camera_vector_t> a;
  size_t m_num_pixel_observations;

public:
  FilterBAModel( std::vector<boost::shared_ptr<CameraModel> > const& cameras,
                 boost::shared_ptr<ba::ControlNetwork> network ) : m_cameras(cameras), m_cnet(network ), m_num_pixel_observations(0), a( cameras.size() ) {
    for ( size_t i = 0; i < m_cnet->size(); ++i )
      m_num_pixel_observations += (*m_cnet)[i].size();

    // Setting up A vector
    for ( size_t j = 0; j < a.size(); j++ )
      a[j] = camera_vector_t();
  }

  // -- REQUIRED STUFF ---------
  Vector2 operator() ( size_t /*i*/, size_t j,
                       camera_vector_t const& a_j,
                       point_vector_t const& b_i ) const {
    boost::shared_ptr<IsisAdjustCameraModel> cam =
      boost::shared_dynamic_cast<IsisAdjustCameraModel>( m_cameras[j] );

    boost::shared_ptr<asp::BaseEquation> posF = cam->position_func();
    boost::shared_ptr<asp::BaseEquation> poseF = cam->pose_func();

    // Apply new constants
    (*posF)[0] = a_j[0];
    (*posF)[1] = a_j[1];
    (*posF)[2] = a_j[2];
    (*poseF)[0] = a_j[3];
    (*poseF)[1] = a_j[4];
    (*poseF)[2] = a_j[5];

    return cam->point_to_pixel( b_i );
  }

  inline Matrix<double,6,6> A_inverse_covariance( size_t /*j*/ ) {
    return 3000*identity_matrix(6);
  }
  inline Matrix<double,3,3> B_inverse_covariance( size_t /*j*/ ) {
    return 10*identity_matrix(3);
  }

  size_t num_cameras() const { return a.size(); }
  size_t num_points() const{ return m_cnet->size(); }
  camera_vector_t A_parameters( size_t j ) const { return a[j]; }
  point_vector_t B_parameters( size_t i ) const { return (*m_cnet)[i].position(); }
  camera_vector_t A_target( size_t j ) const { return camera_vector_t(); }
  point_vector_t B_target( size_t i ) const { return (*m_cnet)[i].position(); }
  size_t num_pixel_observations() const { return m_num_pixel_observations; }
  void set_A_parameters( size_t j, camera_vector_t const& a_j) { a[j] = a_j; }
  void set_B_parameters( size_t i, point_vector_t const& b_i) { (*m_cnet)[i].set_position(b_i); }

  boost::shared_ptr<ba::ControlNetwork> control_network(void) {
    return m_cnet;
  }

  boost::shared_ptr< vw::camera::IsisAdjustCameraModel >
  adjusted_camera( int j ) const {
    boost::shared_ptr<IsisAdjustCameraModel> cam =
      boost::shared_dynamic_cast<IsisAdjustCameraModel>( m_cameras[j] );

    boost::shared_ptr<asp::BaseEquation> posF = cam->position_func();
    boost::shared_ptr<asp::BaseEquation> poseF = cam->pose_func();

    // Apply new constants
    (*posF)[0] = a[j][0];
    (*posF)[1] = a[j][1];
    (*posF)[2] = a[j][2];
    (*poseF)[0] = a[j][3];
    (*poseF)[1] = a[j][4];
    (*poseF)[2] = a[j][5];
    return cam;
  }
  std::vector< boost::shared_ptr< vw::camera::CameraModel > >
  adjusted_cameras() const {
    std::vector< boost::shared_ptr<vw::camera::CameraModel> > cameras;
    for ( unsigned j = 0; j < m_cameras.size(); j++)
      cameras.push_back( adjusted_camera(j) );
    return cameras;
  }

  std::string image_unit() const { return "px"; }
  inline double image_compare( vw::Vector2 const& meas,
                               vw::Vector2 const& obj ) {
    return norm_2(meas-obj);
  }
  inline double position_compare( camera_vector_t const& meas,
                                  camera_vector_t const& obj ) {
    return norm_2(subvector(meas,0,3)-subvector(obj,0,3));
  }
  inline double pose_compare( camera_vector_t const& meas,
                              camera_vector_t const& obj ) {
    return norm_2(subvector(meas,3,3)-subvector(obj,3,3));
  }
  inline double gcp_compare( point_vector_t const& meas,
                             point_vector_t const& obj ) {
    return norm_2(meas-obj);
  }
};

template <class AdjusterT>
void do_ba( FilterBAModel& ba_model ) {
  AdjusterT bundle_adjuster( ba_model, ba::L2Error(),
                             false, false );
  ba::BundleAdjustReport< AdjusterT > reporter( "ba_filter", ba_model,
                                                bundle_adjuster, 27 );

  double abs_tol = 1e10, rel_tol = 1e10;
  double overall_delta = 2;
  size_t no_improvement_count = 0;
  while ( true ) {
    if ( bundle_adjuster.iterations() >= 100 ) {
      reporter() << "Triggered 'Max Iterations'\n";
      break;
    } else if ( abs_tol < 0.01 ) {
      reporter() << "Triggered 'Abs Tol " << abs_tol << " < 0.01'\n";
      break;
    } else if ( abs_tol < 1e-6 ) {
      reporter() << "Triggered 'Rel Tol " << rel_tol << " < 1e-6'\n";
      break;
    } else if ( no_improvement_count > 4 ) {
      reporter() << "Triggered break, unable to improve after "
                 << no_improvement_count << " iterations\n";
      break;
    }

    overall_delta = bundle_adjuster.update( abs_tol, rel_tol );
    if ( overall_delta < 1e-8 )
      no_improvement_count++;
    else
      no_improvement_count = 0;
  }
  reporter.end_tie_in();
}

int main( int argc, char** argv ) {

  try {
    std::string left_image, right_image;
    double threshold;

    po::options_description general_options("Options");
    general_options.add_options()
      ("help,h", "Display this help message")
      ("robust-sparse", "Robust sparse")
      ("dry-run", "Don't save the change")
      ("threshold,t", po::value(&threshold)->default_value(5), "Threshold in pixels for clipping");

    po::options_description hidden_options("");
    hidden_options.add_options()
      ("left-image", po::value(&left_image))
      ("right-image", po::value(&right_image));

    po::options_description options("Allowed Options");
    options.add(general_options).add(hidden_options);

    po::positional_options_description p;
    p.add("left-image", 1);
    p.add("right-image", 1);

    std::ostringstream usage;
    usage << "Usage: " << argv[0] << " [options] <left image> <right image>...\n\n";
    usage << general_options << std::endl;

    po::variables_map vm;
    try {
      po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
      po::notify( vm );
    } catch (po::error &e) {
      std::cout << "An error occured while parsing command line arguments.\n";
      std::cout << "\t" << e.what() << "\n\n";
      std::cout << usage.str();
      return 1;
    }

    if( vm.count("help") ) {
      vw_out() << usage.str();
      return 1;
    }

    if (left_image.empty() || right_image.empty() ) {
      vw_out() << "Error:o Must specify both input files!\n\n";
      vw_out() << usage.str();
      return 1;
    }

    // Replace extensions to cube files
    left_image = fs::path(left_image).replace_extension("cub").string();
    right_image = fs::path(right_image).replace_extension("cub").string();

    // Loading the camera models.
    std::vector< boost::shared_ptr<CameraModel> > camera_models;
    std::vector< std::string > input_names;
    {
      boost::shared_ptr<asp::BaseEquation> posF( new asp::PolyEquation( 0 ) );
      boost::shared_ptr<asp::BaseEquation> poseF( new asp::PolyEquation( 0 ) );
      boost::shared_ptr<CameraModel> p( new IsisAdjustCameraModel( left_image,
                                                                   posF, poseF ));
      camera_models.push_back(p);
      input_names.push_back( left_image );
    }
    {
      boost::shared_ptr<asp::BaseEquation> posF( new asp::PolyEquation( 0 ) );
      boost::shared_ptr<asp::BaseEquation> poseF( new asp::PolyEquation( 0 ) );
      boost::shared_ptr<CameraModel> p( new IsisAdjustCameraModel( right_image,
                                                                   posF, poseF ));
      camera_models.push_back(p);
      input_names.push_back( right_image );
    }

    // Creating Control Network
    boost::shared_ptr<ba::ControlNetwork> cnet( new ba::ControlNetwork("ba_filter") );
    ba::build_control_network( *cnet, camera_models, input_names, 5 );

    VW_ASSERT( cnet->size() != 0, vw::IOErr() << "No matches loaded" );

    // Creating Model
    FilterBAModel ba_model( camera_models, cnet );

    if ( vm.count("robust-sparse") )
      do_ba<ba::AdjustRobustSparse< FilterBAModel, ba::L2Error> >( ba_model );
    else
      do_ba<ba::AdjustSparse< FilterBAModel, ba::L2Error> >( ba_model );

    // Writing out current errors
    std::vector<double> errors( cnet->size() );
    std::fill( errors.begin(), errors.end(), 0 );
    for ( size_t i = 0; i < cnet->size(); i++ ) {
      BOOST_FOREACH( ba::ControlMeasure& cm, (*cnet)[i] ) {
        Vector2 reprojection = ba_model( i, cm.image_id(),
                                         ba_model.A_parameters( cm.image_id() ),
                                         ba_model.B_parameters( i ) );
        errors[i] += norm_2( reprojection - cm.position() );
      }
      errors[i] /= (*cnet)[i].size();
    }

    // Loading interest points
    typedef std::vector< ip::InterestPoint > IPVector;
    IPVector left_ip, right_ip;
    std::string output_filename =
      fs::path(left_image).replace_extension("").string() + "__" +
      fs::path(right_image).replace_extension("").string() + ".match";
    read_binary_match_file( output_filename, left_ip, right_ip );

    // Clipping .. unfortunately we can't assume that the match file and
    // the cnet have the same ordering.
    size_t delete_count = 0;
    for ( size_t cnet_index = 0; cnet_index < cnet->size(); cnet_index++ ) {
      if ( errors[cnet_index] > threshold ) {
        bool found = false;
        for ( size_t ip_index = 0; ip_index < left_ip.size() && !found; ip_index++ ) {
          Vector2 test( left_ip[ip_index].x, left_ip[ip_index].y );
          if ( test == (*cnet)[cnet_index][0].position() ||
               test == (*cnet)[cnet_index][1].position() ) {
            delete_count++;
            IPVector::iterator l_it = left_ip.begin(), r_it = right_ip.begin();
            l_it += ip_index;
            r_it += ip_index;
            left_ip.erase( l_it ); right_ip.erase( r_it );
            found = true;
            break;
          }
        }
        if ( !found )
          vw_out() << "Couldn't find measure to remove. WTF?\n";
      }
    }
    vw_out() << "Removed " << delete_count << " matches\n";

    // Writing back out
    if ( !vm.count("dry-run") )
      write_binary_match_file( output_filename, left_ip, right_ip );
  } ASP_STANDARD_CATCHES;

}
