/// \file ipmatch.cc
///
/// Finds the interest points in an image and outputs them an Binary
/// (default) or ASCII format.  The ASCII format is compatible with
/// the popular Lowe-SIFT toolchain.
///
#include <vw/InterestPoint.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/Camera/CameraGeometry.h>
#include "ann_matcher.h"

using namespace vw;

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

int main(int argc, char** argv) {
  std::vector<std::string> input_file_names;
  double matcher_threshold;
  int pass1_region;
  std::string matrix_string;
  Matrix<float,3,3> starting_transform;
  double inlier_threshold = 20;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message")
    ("pass1-region", po::value(&pass1_region)->default_value(100),
     "Region size in pixels to find a match in the second image.")
    ("matcher-threshold,t", po::value(&matcher_threshold)->default_value(0.6), "Threshold for the interest point matcher.");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-matrix", po::value(&matrix_string))
    ("input-files", po::value(&input_file_names));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-matrix", 1);
  p.add("input-files", -1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <filenames>..." << std::endl << std::endl;
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

  if( input_file_names.size() < 2 ) {
    vw_out() << "Error: Must specify at least two input files!" << std::endl << std::endl;
    vw_out() << usage.str();
    return 1;
  }

  // Parse matrix string
  {
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(",:;{}[] ");
    tokenizer tokens( matrix_string, sep );
    Matrix<float,3,3>::iterator mat_iter = starting_transform.begin();
    for ( tokenizer::iterator tok_iter = tokens.begin();
          tok_iter != tokens.end(); tok_iter++ ) {
      if ( mat_iter == starting_transform.end() )
        vw_throw( ArgumentErr() << "Input matrix string has incorrect number of elements. I need a 3x3 matrix." );
      *mat_iter = boost::lexical_cast<float>(*tok_iter);
      mat_iter++;
    }
    if ( mat_iter != starting_transform.end() )
      vw_throw( ArgumentErr() << "Input matrix string has incorrect number of elements. I need a 3x3 matrix." );
    vw_out() << "\t--> Matrix: " << starting_transform << "\n";
  }

  // Read each file off disk
  typedef std::vector<ip::InterestPoint> IPVector;
  IPVector ip1, ip2;
  ip1 = ip::read_binary_ip_file(fs::path(input_file_names[0]).replace_extension("vwip").string() );
  ip2 = ip::read_binary_ip_file(fs::path(input_file_names[1]).replace_extension("vwip").string() );
  vw_out() << "Matching between " << input_file_names[0] << " (" << ip1.size() << " points) and " << input_file_names[1] << " (" << ip2.size() << " points).\n";

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

  // Iterate over combinations of the input files and find interest
  // points in each.
  std::vector<int> matched_index( ip1.size() );
  TerminalProgressCallback tpc("apollo","Pass 1:");
  double inc_amt = 1.0/float(ip1.size());
  count = 0;
  BOOST_FOREACH( ip::InterestPoint const& ip, ip1 ) {
    tpc.report_incremental_progress( inc_amt );
    Vector3f input(ip.x,ip.y,1);
    Vector3f output = starting_transform*input;
    output /= output[2];
    Vector2f query(output[0],output[1]);

    double sqRadius = pass1_region*pass1_region;
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
    if ( distance1 < distance2*matcher_threshold ) {
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
  Matrix<double> H;
  IPVector final_ip1, final_ip2;
  {
    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
    std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
    std::vector<int> indices;

    // RANSAC is used to fit a transform between the matched sets
    // of points.  Points that don't meet this geometric
    // contstraint are rejected as outliers.
    typedef math::HomographyFittingFunctor fit_func;
    typedef math::InterestPointErrorMetric err_func;
    math::RandomSampleConsensus<fit_func, err_func> ransac( fit_func(),
                                                            err_func(),
                                                            inlier_threshold ); // inlier_threshold
    Matrix<double> H(ransac(ransac_ip1,ransac_ip2));
    std::cout << "\t--> Homography: " << H << "\n";
    indices = ransac.inlier_indices(H,ransac_ip1,ransac_ip2);

    for (unsigned idx=0; idx < indices.size(); ++idx) {
      final_ip1.push_back(matched_ip1[indices[idx]]);
      final_ip2.push_back(matched_ip2[indices[idx]]);
    }
  }
  std::cout << "Found " << final_ip1.size() << " matches after RANSAC.\n";

  // Write out result
  std::string output_filename =
    fs::path(input_file_names[0]).replace_extension("").string() + "__" +
    fs::path(input_file_names[1]).replace_extension("").string() + ".match";
  std::cout << "Writing: " << output_filename << "\n";
  ip::write_binary_match_file( output_filename, final_ip1, final_ip2 );

  // Deallocate kd tree
  annDeallocPts( ann_pts );
  delete kdtree_ispace2;
  annClose();
}
