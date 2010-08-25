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
#include "ransac.h"
#include "ann_matcher.h"

using namespace vw;
using namespace vw::ip;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

int main(int argc, char** argv) {
  std::vector<std::string> input_file_names;
  double matcher_threshold;
  int inlier_threshold = 20;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message")
    ("fundamental-matrix", "Use a fundamental matrix fitting")
    ("matcher-threshold,t", po::value<double>(&matcher_threshold)->default_value(0.6), "Threshold for the interest point matcher.");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_file_names));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
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

  // Iterate over combinations of the input files and find interest
  // points in each.
  for (unsigned i = 0; i < input_file_names.size(); ++i) {
    for (unsigned j = i+1; j < input_file_names.size(); ++j) {

      // Read each file off disk
      std::vector<InterestPoint> ip1, ip2;
      ip1 = read_binary_ip_file(fs::path(input_file_names[i]).replace_extension("vwip").string() );
      ip2 = read_binary_ip_file(fs::path(input_file_names[j]).replace_extension("vwip").string() );
      vw_out() << "Matching between " << input_file_names[i] << " (" << ip1.size() << " points) and " << input_file_names[j] << " (" << ip2.size() << " points).\n";

      std::vector<InterestPoint> matched_ip1, matched_ip2;

      // Run interest point matcher that does not use KDTree algorithm.
      //InterestPointMatcherSimple<L2NormMetric,NullConstraint> matcher(matcher_threshold);
      InterestPointMatcherANN matcher( matcher_threshold );
      matcher(ip1, ip2, matched_ip1, matched_ip2, false,
              TerminalProgressCallback( "tools.ipmatch","Matching:"));

      remove_duplicates(matched_ip1, matched_ip2);
      vw_out() << "Found " << matched_ip1.size() << " putative matches.\n";

      std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
      std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
      std::vector<int> indices;
      try {
        if ( vm.count("fundamental-matrix") ) {
          typedef camera::FundamentalMatrix8PFittingFunctor fit_func;
          typedef camera::FundamentalMatrixDistanceErrorMetric err_func;
          math::RandomSampleConsensus<fit_func, err_func> ransac( fit_func(),
                                                                  err_func(),
                                                                  5 );
          Matrix<double> F(ransac(ransac_ip1,ransac_ip2));
          std::cout << "\t--> Fundamental: " << F << "\n";
          indices = ransac.inlier_indices(F,ransac_ip1,ransac_ip2);
        } else {
          // RANSAC is used to fit a transform between the matched sets
          // of points.  Points that don't meet this geometric
          // contstraint are rejected as outliers.
          typedef math::HomographyFittingFunctor fit_func;
          typedef math::InterestPointErrorMetric err_func;
          math::RandomSampleConsensusMod<fit_func, err_func> ransac( fit_func(),
                                                                     err_func(),
                                                                     inlier_threshold ); // inlier_threshold
          Matrix<double> H(ransac(ransac_ip1,ransac_ip2));
          std::cout << "\t--> Homography: " << H << "\n";
          indices = ransac.inlier_indices(H,ransac_ip1,ransac_ip2);
        }
      } catch (vw::math::RANSACErr &e) {
        std::cout << "RANSAC Failed: " << e.what() << "\n";
        continue;
      }
      vw_out() << "Found " << indices.size() << " final matches.\n";

      std::vector<InterestPoint> final_ip1, final_ip2;
      for (unsigned idx=0; idx < indices.size(); ++idx) {
        final_ip1.push_back(matched_ip1[indices[idx]]);
        final_ip2.push_back(matched_ip2[indices[idx]]);
      }

      std::string output_filename =
        fs::path(input_file_names[i]).stem() + "__" +
        fs::path(input_file_names[j]).stem() + ".match";
      write_binary_match_file(output_filename, final_ip1, final_ip2);
    }
  }
}

