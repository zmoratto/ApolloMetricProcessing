/// Kriging Guided Match
///
/// This requires some matches to already to exist

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include "Kriging.h"
#include "ann_matcher.h"

using namespace vw;

#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

std::string left, right;
float search_scalar, matcher_threshold;

template <class T>
void filter_vwip( T& subject, T& search_terms ) {
  for ( typename T::iterator i = subject.begin();
        i != subject.end(); i++ ) {
    bool found = false;
    for ( typename T::const_iterator j = search_terms.begin();
          j != search_terms.end(); j++ ) {
      if ( i->x == j->x && i->y == j->y ) {
        found = true;
        break;
      }
    }

    if ( found ) {
      i = subject.erase( i );
      i--;
    }
  }
}

void do_guided_search() {
  // Loading previos matches
  typedef std::vector< ip::InterestPoint > IPVector;
  IPVector matched_ip1, matched_ip2;
  std::string match_filename =
    fs::path(left).replace_extension("").string() + "__" +
    fs::path(right).replace_extension("").string() + ".match";
  if (!fs::exists(match_filename))
    vw_throw( ArgumentErr() << "Input images don't already have matches to seed the search." );
  ip::read_binary_match_file(match_filename, matched_ip1, matched_ip2);
  vw_out() << "Read " << matched_ip1.size() << " matches from match file.\n";

  // Loading up VWIP and removing already matched ips
  IPVector vwip_ip1, vwip_ip2;
  vwip_ip1 = ip::read_binary_ip_file(fs::path(left).replace_extension("vwip").string() );
  vwip_ip2 = ip::read_binary_ip_file(fs::path(right).replace_extension("vwip").string() );
  filter_vwip( vwip_ip1, matched_ip1 );
  filter_vwip( vwip_ip2, matched_ip2 );
  vw_out() << "Found " << vwip_ip1.size() << " and " << vwip_ip2.size() << " point remaining to be matched.\n";

  // Producing Kriging
  typedef std::pair<Vector2f, Vector2f> pair_type;
  std::list<pair_type> samples;
  for ( size_t i = 0; i < matched_ip1.size(); i++ ) {
    Vector2f lloc( matched_ip1[i].x, matched_ip1[i].y ),
      rloc( matched_ip2[i].x, matched_ip2[i].y );
    samples.push_back( pair_type(lloc,rloc-lloc) );
  }
  KrigingView<Vector2f> disparity( samples, BBox2i() );

  // Building ANN kdtree in image search for vwip_ip2
  ANNpointArray ann_pts;
  ann_pts = annAllocPts( vwip_ip2.size(), 2 );
  int count = 0;
  for ( IPVector::iterator ip = vwip_ip2.begin();
        ip != vwip_ip2.end(); ip++ ) {
    ann_pts[count][0] = ip->x;
    ann_pts[count][1] = ip->y;
    count++;
  }
  ANNkd_tree* kdtree_ispace2 = new ANNkd_tree( ann_pts, vwip_ip2.size(), 2 );

  // Iterate over combinations of the input files and find interest points
  std::vector<int> matched_index( vwip_ip1.size() );
  TerminalProgressCallback tpc("tools", "Matching:");
  double inc_amt = 1.0/double(vwip_ip2.size());
  count = 0;
  BOOST_FOREACH( ip::InterestPoint const& ip, vwip_ip1 ) {
    tpc.report_incremental_progress( inc_amt );
    Vector2f input(ip.x,ip.y);
    Vector2f error;
    Vector2f query = input + disparity.error( ip.x, ip.y, error );

    double sqRadius = pow(norm_2(error)*search_scalar,2);
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
      double dist = norm_2_sqr( ip.descriptor - vwip_ip2[*index].descriptor );
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

  // Writing first pass matches
  size_t num_added = 0;
  for ( size_t i = 0; i < matched_index.size(); i++ ) {
    if ( matched_index[i] >= 0 ) {
        num_added++;
        matched_ip1.push_back( vwip_ip1[i] );
        matched_ip2.push_back( vwip_ip2[matched_index[i]] );
    }
  }
  vw_out() << "Added " << num_added << " matches from Kriging.\n";
  vw_out() << matched_ip1.size() << " matches total.\n";
  vw_out() << "Writing: " << match_filename << "\n";
  ip::write_binary_match_file( match_filename, matched_ip1, matched_ip2 );

  // Deallocate kd tree
  annDeallocPts( ann_pts );
  delete kdtree_ispace2;
  annClose();
}

int main(int argc, char** argv) {

  po::options_description general_options("Options");
  general_options.add_options()
    ("scalar,s", po::value(&search_scalar)->default_value(10),
     "Scalar to apply to kriging interpolation error." )
    ("matcher-threshold,t", po::value(&matcher_threshold)->default_value(0.6), "Threshold for the interest point matcher.")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("left-image", po::value(&left))
    ("right-image", po::value(&right));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("left-image", 1);
  p.add("right-image", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <filenames>...\n\n";
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_out() << "An error occured while parsing command line arguments.\n";
    vw_out() << "\t" << e.what() << "\n\n";
    vw_out() << usage.str();
    return 1;
  }

  if( vm.count("help") ) {
    vw_out() << usage.str();
    return 1;
  }

  if ( left.empty() || right.empty() ) {
    vw_out() << "Error: Must specify at least two input files!" << std::endl << std::endl;
    vw_out() << usage.str();
    return 1;
  }

  try {
    do_guided_search();
  } catch ( const ArgumentErr& e ) {
    vw_out() << e.what() << std::endl;
    return 1;
  } catch ( const Exception& e ) {
    std::cerr << "\n\nVW Error: " << e.what() << std::endl;
    return 1;
  } catch ( const std::bad_alloc& e ) {
    std::cerr << "\n\nError: Ran out of Memory!" << std::endl;
    return 1;
  } catch ( const std::exception& e ) {
    std::cerr << "\n\nError: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

