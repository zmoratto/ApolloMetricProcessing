#include <vw/Core/ProgressCallback.h>
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/Camera/CameraGeometry.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Cartography/Datum.h>
#include <asp/IsisIO.h>
#include <asp/Core/Macros.h>

#include <ANN/ANN.h>

using namespace vw;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

struct Options {
  // Inputs
  std::string cube_file1, cube_file2, cam_file1, cam_file2;
  std::string datum;

  // Settings
  double matcher_threshold, pass1_region;
  BBox2i image1, image2;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  std::vector<std::string> input_files;
  po::options_description general_options("");
  general_options.add_options()
    ("reference-spheroid,r", po::value(&opt.datum)->default_value("moon"),
     "Set a reference surface to a hard coded value (one of [moon, mars, wgs84].)")
    ("matcher-threshold,t", po::value(&opt.matcher_threshold)->default_value(0.6), "Threshold for the interest point matcher.")
    ("pass1-region", po::value(&opt.pass1_region)->default_value(100),
     "Region size in pixels to find a match in the second image.")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&input_files) );

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e ) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <cube 1> \"opt. adjust\" <cube 2> \"opt. adjust\"";
  usage << "Note: All cameras and their images must be of the same session type. Camera models only can be used as input for stereo sessions pinhole and isis.\n\n";

  // Determining if feed only camera model
  if ( input_files.size() == 2 ) {
    opt.cube_file1 = input_files[0];
    opt.cube_file2 = input_files[1];
  } else if ( input_files.size() == 4 ) {
    opt.cube_file1 = input_files[0];
    opt.cube_file2 = input_files[2];
    opt.cam_file1 = input_files[1];
    opt.cam_file2 = input_files[3];
  } else {
    vw_throw( ArgumentErr() << "Incorrect number of input arguments. Expected 2 camera files, or 2 camera files plus 2 adjustments.\n\n" << usage.str() << general_options );
  }

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
}

void read_isis_adjusted( std::string const& image_file,
                         std::string const& camera_file,
                         boost::shared_ptr<camera::CameraModel>& cam ) {
  std::ifstream input( camera_file.c_str() );
  boost::shared_ptr<asp::BaseEquation> posF = asp::read_equation( input );
  boost::shared_ptr<asp::BaseEquation> poseF = asp::read_equation( input );
  input.close();

  cam = boost::shared_ptr<camera::CameraModel>( new camera::IsisAdjustCameraModel(image_file, posF, poseF) );
}

Vector3 sphere_intersection( boost::shared_ptr<camera::CameraModel> cam,
                              Vector2 const& query,
                              cartography::Datum const& datum ) {
  Vector3 c = -cam->camera_center( query ); // Sphere center
  Vector3 l = cam->pixel_to_vector( query );
  double r = datum.semi_major_axis();

  // uner sqrt
  double under_sqrt = pow(dot_prod(l,c),2)-norm_2_sqr(c)+r*r;
  if ( under_sqrt < 0 )
    return Vector3();
  if ( fabs(under_sqrt) < 1e-6 ) {
    return dot_prod(l,c)+sqrt(under_sqrt);
  } else {
    double sol1 = dot_prod(l,c)+sqrt(under_sqrt);
    double sol2 = dot_prod(l,c)-sqrt(under_sqrt);
    if ( sol1 < sol2 )
      return -c+sol1*l;
    else
      return -c+sol2*l;
  }
}

int main(int argc, char** argv) {
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Reading Camera Models
    boost::shared_ptr<camera::CameraModel> cam1, cam2;
    if ( opt.cam_file1.empty() && opt.cam_file2.empty() ) {
      typedef camera::IsisCameraModel CamType;
      cam1 = boost::shared_ptr<camera::CameraModel>( new CamType(opt.cube_file1) );
      cam2 = boost::shared_ptr<camera::CameraModel>( new CamType(opt.cube_file2) );
      boost::shared_ptr<CamType> upcast1, upcast2;
      upcast1 = boost::shared_dynamic_cast<CamType>( cam1 );
      upcast2 = boost::shared_dynamic_cast<CamType>( cam2 );
      opt.image1 = BBox2i( 0, 0, upcast1->samples(), upcast2->lines() );
      opt.image2 = BBox2i( 0, 0, upcast2->samples(), upcast2->lines() );
    } else {
      typedef camera::IsisAdjustCameraModel CamType;
      read_isis_adjusted( opt.cube_file1, opt.cam_file1, cam1 );
      read_isis_adjusted( opt.cube_file2, opt.cam_file2, cam2 );
      if ( !cam1 )
        std::cout << "Cam1 not loaded\n";
      boost::shared_ptr<CamType> upcast1, upcast2;
      upcast1 = boost::shared_dynamic_cast<CamType>( cam1 );
      upcast2 = boost::shared_dynamic_cast<CamType>( cam2 );
      opt.image1 = BBox2i( 0, 0, upcast1->samples(), upcast2->lines() );
      opt.image2 = BBox2i( 0, 0, upcast2->samples(), upcast2->lines() );
    }

    // Read inputs
    typedef std::vector<ip::InterestPoint> IPVector;
    IPVector ip1, ip2;
    ip1 = ip::read_binary_ip_file( fs::path(opt.cube_file1).replace_extension("vwip").string() );
    ip2 = ip::read_binary_ip_file( fs::path(opt.cube_file2).replace_extension("vwip").string() );

    std::cout << "Matching between " << opt.cube_file1 << " (" << ip1.size() << " points) and " << opt.cube_file2 << " (" << ip2.size() << " points).\n";

    // Load up datum
    cartography::Datum datum;
    if ( opt.datum == "mars" ) {
      datum.set_well_known_datum("D_MARS");
    } else if ( opt.datum == "moon" ) {
      datum.set_well_known_datum("D_MOON");
    } else if ( opt.datum == "wgs84" ) {
      datum.set_well_known_datum("WGS84");
    } else {
      vw_out() << "Unknown spheriod request: " << opt.datum << "\n";
      vw_out() << "->  Defaulting to WGS84\n";
      datum.set_well_known_datum("WGS84");
    }

    // Build ANN kdtree in image space for ip2
    ANNpointArray ann_pts;
    ann_pts = annAllocPts( ip2.size(), 2 );
    int count = 0;
    for ( IPVector::iterator ip = ip2.begin();
          ip < ip2.end(); ip++ ) {
      ann_pts[count][0] = ip->x;
      ann_pts[count][1] = ip->y;
      count++;
    }
    ANNkd_tree* kdtree_ispace2 = new ANNkd_tree( ann_pts, ip2.size(), 2 );

    // Matching individual ips in image1 to image2
    TerminalProgressCallback tpc("apollo","Pass 1:");
    double inc_amt = 1.0/float(ip1.size());
    std::vector<int> matched_index( ip1.size() );
    count = 0;
    BOOST_FOREACH( ip::InterestPoint const& ip, ip1 ) {
      tpc.report_incremental_progress( inc_amt );
      Vector3 moon_intersect =
        sphere_intersection( cam1, Vector2(ip.x,ip.y), datum );
      if ( moon_intersect == Vector3() ) {
        matched_index[count] = -1;
        count++;
        continue;
      }
      Vector2f query = cam2->point_to_pixel( moon_intersect );
      if ( !opt.image2.contains( query ) ) {
        matched_index[count] = -1;
        count++;
        continue;
      }

      double sqRadius = opt.pass1_region*opt.pass1_region;
      // Find out the number of points in this area
      int k = kdtree_ispace2->annkFRSearch( &query[0], sqRadius, 0,
                                            NULL, NULL, 0.0 );
      if ( k < 2 ) {
        matched_index[count] = -1;
        count++;
        continue;
      }

      // Find indices of nearest point
      std::vector<int> found_indices( k );
      std::vector<double> found_distances( k );
      kdtree_ispace2->annkFRSearch( &query[0], sqRadius, k,
                                    &found_indices[0],
                                    &found_distances[0], 0.0 );

      // Iterate to find the 2 closests points
      double distance1, distance2;
      distance1 = distance2 = std::numeric_limits<double>::max();
      int best_index = -1;
      for ( std::vector<int>::iterator index = found_indices.begin();
            index < found_indices.end(); index++ ) {
        double dist = norm_2_sqr( ip.descriptor - ip2[*index].descriptor );
        if ( dist < distance1 ) {
          best_index = *index;
          distance2 = distance1;
          distance1 = dist;
        } else if ( dist < distance2 ) {
          distance2 = dist;
        }
      }

      // Determine if we have a decent match
      if ( distance1 < distance2*opt.matcher_threshold ) {
        matched_index[count] = best_index;
      } else {
        matched_index[count] = -1;
      }

      count++;
    }
    tpc.report_finished();

    // Write first pass matches
    IPVector matched_ip1, matched_ip2;
    {
      int count = 0;
      for ( std::vector<int>::iterator index = matched_index.begin();
            index < matched_index.end(); index++ ) {
        if ( *index < 0 ) {
          count++;
          continue;
        }
        matched_ip1.push_back( ip1[count] );
        matched_ip2.push_back( ip2[*index] );
        count++;
      }
    }
    std::cout << "Found " << matched_ip1.size() << " matches on first pass.\n";

    // Fit a Fundamental Matrix
    Matrix<double> F;
    IPVector final_ip1, final_ip2;
    {
      std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
      std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
      std::vector<int> indices;

      typedef camera::FundamentalMatrix8PFittingFunctor fit_func;
      typedef camera::FundamentalMatrixDistanceErrorMetric err_func;
      math::RandomSampleConsensus<fit_func, err_func> ransac( fit_func(),
                                                              err_func(),
                                                              5 );
      F = ransac(ransac_ip1,ransac_ip2);
      std::cout << "\t--> Fundamental: " << F << "\n";
      indices = ransac.inlier_indices(F,ransac_ip1,ransac_ip2);

      for (unsigned idx=0; idx < indices.size(); ++idx) {
        final_ip1.push_back(matched_ip1[indices[idx]]);
        final_ip2.push_back(matched_ip2[indices[idx]]);
      }
    }
    std::cout << "Found " << final_ip1.size() << " matches after RANSAC.\n";

    // Perform 2nd pass
    //      I don't know if I need it3

    // Write out result
    std::string output_filename =
      fs::path(opt.cube_file1).replace_extension("").string() + "__" +
      fs::path(opt.cube_file2).replace_extension("").string() + ".match";
    std::cout << "Writing: " << output_filename << "\n";
    write_binary_match_file( output_filename, final_ip1, final_ip2 );

    // Deallocate kd tree
    annDeallocPts( ann_pts );
    delete kdtree_ispace2;
    annClose();

  } ASP_STANDARD_CATCHES;

  return 0;
};
