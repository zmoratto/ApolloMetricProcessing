// Zack Moratto
// Meant to be used with cube files with no other information.
// *WARNING* This has been modifed from the original used on the super computer
// to use only VWIP stuff.

// This project
#include "overlap_check.h"
#include "surf_io.h"
#include "equalization.h"
#include "RANSAC_mod.h"

// std header
#include <stdlib.h>
#include <iostream>
#include <vector>

// Boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

// Vision Workbench
#include <vw/Math.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/Matcher.h>
using namespace vw;
using namespace vw::ip;

// Suffix
static std::string replace_suffix(std::string const& filename,
                                  std::string const& new_suffix ) {
  std::string result = filename;
  int index = result.rfind(".");
  if ( index != -1 )
    result.erase(index+1, result.size());
  result.append( new_suffix );
  return result;
}

// Remove Directory
static std::string remove_dir_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind("/");
  if (index != -1)
    result.erase(0,index+1);
  return result;
}

// Prefix (this also strips directory info)
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

int main(int argc, char* argv[]) {
  float req_percent_overlap;
  std::string left_cube;
  std::string right_cube;
  int max_points;

  // Boost Program Options code
  po::options_description general_options("Options");
  general_options.add_options()
    ("overlap,o",po::value<float>(&req_percent_overlap)->default_value(20),"Minimium requirement for image overlap")
    ("max-pts,m",po::value<int>(&max_points)->default_value(200),"Max points a pair can have, if the number of matches exceeds it will be widdled down by space equalization.")
    ("help,h", "Display this help message");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-cube-image",po::value<std::string>(&left_cube), "Left cube input")
    ("right-cube-image",po::value<std::string>(&right_cube),"Right cube input");

  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-cube-image", 1);
  positional_options_desc.add("right-cube-image",1);

  po::options_description all_options;
  all_options.add(general_options).add(positional_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_options_desc).run(), vm );
  po::notify(vm);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <left cube> <right cube> [options] ... " << std::endl;
  usage << general_options << std::endl;

  if ( vm.count("help")) {
    std::cout << usage.str() << std::endl;
    exit(0);
  }

  if ( !vm.count("left-cube-image") || !vm.count("right-cube-image") ) {
    std::cout << "\n Missing input cube files.\n";
    std::cout << usage.str() << std::endl;
    exit(0);
  }

  double overlap = percent_overlap( left_cube,
                                    right_cube );

  if ( overlap >= req_percent_overlap ) {
    std::cout << "Overlap = [TRUE] [" << overlap << "%]\n";
  } else {
    std::cout << "Overlap = [FALSE][" << overlap << "%]\n";
    exit(0);
  }

  // Checking VWIP files
  std::string left_vwip = prefix_from_filename(left_cube)+".vwip";
  std::string right_vwip = prefix_from_filename(right_cube)+".vwip";
  if ( !fs::exists(left_vwip) ) {
    std::cout << " -- > Didn't find left vwip: " << left_vwip << "\n";
    std::ostringstream cmd;
    cmd << "ipfind --num 4 --max " << uint(max_points*1.5) << " "
        << prefix_from_filename(left_cube) << ".tif --int obalog --des sgrad -g 1.2";
    std::system( cmd.str().c_str() );
  }
  if ( !fs::exists(right_vwip) ) {
    std::cout << " -- > Didn't find right vwip: " << right_vwip << "\n";
    std::ostringstream cmd;
    cmd << "ipfind --num 4 --max " << uint(max_points*1.5) << " "
        << prefix_from_filename(right_cube) << ".tif --int obalog --des sgrad -g 1.2";
    std::system( cmd.str().c_str() );
  }

  // Loading IP files
  std::vector<InterestPoint> l_ip = read_binary_ip_file( left_vwip );
  std::vector<InterestPoint> r_ip = read_binary_ip_file( right_vwip );

  // Checking for previous match files
  std::string output_filename =
    prefix_from_filename(remove_dir_from_filename(left_cube)) + "__" +
    prefix_from_filename(remove_dir_from_filename(right_cube)) + ".match";
  if ( fs::exists(output_filename) ) {
    std::cout << "Match file already exists:\n";
    exit(0);
  }

  std::vector<InterestPoint> matched_ip1, matched_ip2;
  vw_out() << "Matching between " << left_cube << " (" << l_ip.size() << " points) and " << right_cube << " (" << r_ip.size() << " points).\n";

  // Non-kdtree / Matcher (most like the original SURF)
  InterestPointMatcherSimple<L2NormMetric,NullConstraint> matcher(0.5);
  matcher(l_ip,r_ip,matched_ip1,matched_ip2,false);

  remove_duplicates(matched_ip1, matched_ip2);
  vw_out(InfoMessage) << "Found " << matched_ip1.size() << " putative matches.\n";

  // RANSAC
  vw_out() << "Performing RANSAC to throw out outliers:\n";

  Matrix<double> T;
  try {
    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
    std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);

    math::RandomSampleConsensusMod<math::HomographyFittingFunctor,math::InterestPointErrorMetric> ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 15 );
    T = ransac( ransac_ip1, ransac_ip2 );

    std::vector<int> indices = ransac.inlier_indices(T, ransac_ip1, ransac_ip2 );
    vw_out() << "\t> Found " << indices.size() << " acceptable matches.\n";

    std::vector<InterestPoint> final_ip1, final_ip2;
    for ( unsigned i = 0; i < indices.size(); ++i ) {
      final_ip1.push_back( matched_ip1[ indices[i] ] );
      final_ip2.push_back( matched_ip2[ indices[i] ] );
    }
    matched_ip1 = final_ip1;
    matched_ip2 = final_ip2;
  } catch(...) {
    vw_out() << "RANSAC Failed!\n";
    vw_out() << "Skipping to next file...\n\n";
    exit(0);
  }

  vw_out() << "Performing equalization\n";
  equalization( matched_ip1,
                matched_ip2, max_points );
  vw_out() << "\t> Reduced matches to " << matched_ip1.size() << " pairs.\n";

  // Finally write the match file
  vw_out() << "Writing: " << output_filename << std::endl;
  write_binary_match_file( output_filename, matched_ip1,
                           matched_ip2 );
}
