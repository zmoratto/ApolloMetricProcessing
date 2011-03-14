/// \file apollo_bulk_match.cc
///
/// Process's vwip files into a bulk match file. The input is a text
/// file of pairs to run. This process will write out a compressed
/// file with all the match results in them.
///
/// This file is meant to do apollo_match and reduce_match in one
/// go. This program exports a large file to get around a limit on
/// number files that the super computer imposes.
///
#include <vw/Core.h>
#include <vw/InterestPoint.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math.h>
#include "ransac.h"

using namespace vw;
using namespace vw::ip;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file.hpp>
namespace io = boost::iostreams;

#include <boost/foreach.hpp>

#include "iprecord.h"

// Duplicate matches for any given interest point probably indicate a
// poor match, so we cull those out here.
void remove_duplicates(std::vector<InterestPoint> &ip1, std::vector<InterestPoint> &ip2) {
  std::vector<InterestPoint> new_ip1, new_ip2;

  for (size_t i = 0; i < ip1.size(); ++i) {
    bool bad_entry = false;
    for (size_t j = 0; j < ip1.size(); ++j) {
      if (i != j &&
          ((ip1[i].x == ip1[j].x && ip1[i].y == ip1[j].y) ||
           (ip2[i].x == ip2[j].x && ip2[i].y == ip2[j].y)) ) {
        bad_entry = true;
      }
    }
    if (!bad_entry) {
      new_ip1.push_back(ip1[i]);
      new_ip2.push_back(ip2[i]);
    }
  }

  ip1 = new_ip1;
  ip2 = new_ip2;
}

// Handles multiple tasks of match and serialize their writing to file.
class ThreadedMatcher : private boost::noncopyable {
  boost::shared_ptr<FifoWorkQueue> m_match_queue;
  boost::shared_ptr<FifoWorkQueue> m_write_queue;
  boost::shared_ptr<io::filtering_ostream> m_filter;

  // --- Task Types (2) ----
  class WriteTask : public Task {
    ThreadedMatcher &m_parent;
    std::string m_left_name, m_right_name;
    std::vector<InterestPoint> m_left, m_right;
  public:
    WriteTask( ThreadedMatcher &parent,
               std::string const& left_name, std::string const& right_name,
               std::vector<InterestPoint> left,
               std::vector<InterestPoint> right ) : m_parent(parent), m_left_name(left_name), m_right_name(right_name), m_left(left), m_right(right) {}

    virtual ~WriteTask(){}
    virtual void operator()() {
      std::cout << "Writing! " << m_left_name << " " << m_right_name << "\n";
      boost::shared_ptr<io::filtering_ostream> out = m_parent.get_filtering_stream();
      std::string header = fs::path(m_left_name).stem()+"__"+fs::path(m_right_name).stem()+".match";
      int header_size = header.size();
      std::cout << "Header size: " << header_size << "\n";
      out->write( (char*)&header_size, sizeof(header_size) );
      out->write( &header[0], header_size );

      // Writing out number left
      int match_size = m_left.size();
      out->write( (char*)&match_size, sizeof(match_size) );
      BOOST_FOREACH( InterestPoint const& ip, m_left ) { write_ip_record( *out, ip ); }

      // Writing out number right
      match_size = m_right.size();
      out->write( (char*)&match_size, sizeof(match_size) );
      BOOST_FOREACH( InterestPoint const& ip, m_right ) { write_ip_record( *out, ip ); }

    }
  };

  class MatchTask : public Task {
    ThreadedMatcher &m_parent;
    std::string m_left, m_right;
    double m_match_threshold;
    int m_inlier_threshold;
  public:
    MatchTask( ThreadedMatcher &parent, std::string const& left, std::string const& right,
               double match_t, int inlier_t ) : m_parent(parent), m_left(left), m_right(right), m_match_threshold(match_t), m_inlier_threshold(inlier_t) {
    }

    virtual ~MatchTask() {}
    virtual void operator()() {
      std::vector<InterestPoint> ip1, ip2;
      ip1 = read_binary_ip_file(fs::path(m_left).replace_extension("vwip").string() );
      ip2 = read_binary_ip_file(fs::path(m_right).replace_extension("vwip").string() );
      vw_out() << "Matching between " << m_left << " (" << ip1.size() << " points) and " << m_right << " (" << ip2.size() << " points).\n";

      std::vector<InterestPoint> matched_ip1, matched_ip2;

      DefaultMatcher matcher( m_match_threshold );
      matcher(ip1, ip2, matched_ip1, matched_ip2, false,
              TerminalProgressCallback( "tools.ipmatch","Matching:"));

      remove_duplicates(matched_ip1, matched_ip2);
      vw_out() << "Found " << matched_ip1.size() << " putative matches.\n";

      std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
      std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
      std::vector<int> indices;
      try {
        // RANSAC is used to fit a transform between the matched sets
        // of points.  Points that don't meet this geometric
        // contstraint are rejected as outliers.
        typedef math::HomographyFittingFunctor fit_func;
        typedef math::InterestPointErrorMetric err_func;
        math::RandomSampleConsensusMod<fit_func, err_func> ransac( fit_func(),
                                                                   err_func(),
                                                                   m_inlier_threshold ); // inlier_threshold
        Matrix<double> H(ransac(ransac_ip1,ransac_ip2));
        std::cout << "\t--> Homography: " << H << "\n";
        indices = ransac.inlier_indices(H,ransac_ip1,ransac_ip2);
      } catch (vw::math::RANSACErr &e) {
        std::cout << "RANSAC Failed: " << e.what() << "\n";
        return;
      }
      vw_out() << "Found " << indices.size() << " final matches.\n";

      if ( indices.size() >= 10 ) {

        std::vector<InterestPoint> final_ip1, final_ip2;
        for (unsigned idx=0; idx < indices.size(); ++idx) {
          final_ip1.push_back(matched_ip1[indices[idx]]);
          final_ip2.push_back(matched_ip2[indices[idx]]);
        }

        // Spawning a write task
        boost::shared_ptr<Task> write_task( new WriteTask( m_parent, m_left, m_right,
                                                           final_ip1, final_ip2 ) );
        m_parent.add_write_task(write_task);
      } else {
        std::cout << "Failed to find enough matches!\n";
        return;
      }
    }
  };

  void add_match_task( boost::shared_ptr<Task> task ) { m_match_queue->add_task(task); }
  void add_write_task( boost::shared_ptr<Task> task ) { m_write_queue->add_task(task); }
  boost::shared_ptr<io::filtering_ostream> get_filtering_stream() { return m_filter; }

public:

  ThreadedMatcher( int num_threads, std::string const& out_file ) {
    m_match_queue = boost::shared_ptr<FifoWorkQueue>( new FifoWorkQueue(num_threads) );
    m_write_queue = boost::shared_ptr<FifoWorkQueue>( new FifoWorkQueue(1) );

    // Open the write file here
    m_filter = boost::shared_ptr<io::filtering_ostream>( new io::filtering_ostream() );
    m_filter->push( io::gzip_compressor() );
    m_filter->push( io::file_sink( out_file ) );

    if ( !m_filter->good() )
      vw_throw( IOErr() << "Stream no good!\n" );

    std::string header = "Packed Match File!";
    int header_size = header.size();
    std::cout << "Header size: " << header_size << "\n";
    m_filter->write( (char*)&header_size, sizeof(header_size) );
    m_filter->write( &header[0], header_size );
  }

  void add_match( std::string const& left, std::string const& right,
                  double match_t, int inlier_t ) {
    boost::shared_ptr<Task> task( new MatchTask( *this, left, right, match_t, inlier_t ) );
    this->add_match_task( task );
  }

  void process_matches() {
    m_match_queue->join_all();
    m_write_queue->join_all();
    m_filter->flush();
  }

};

int main(int argc, char** argv) {
  std::string job_list;
  double matcher_threshold;
  int inlier_threshold = 20;
  int number_threads;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message")
    ("threads", po::value(&number_threads)->default_value(4), "Number of threads to use for matching.\n")
    ("matcher-threshold,t", po::value(&matcher_threshold)->default_value(0.6), "Threshold for the interest point matcher.");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("job-list", po::value(&job_list));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("job-list", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <job_list_file> \n\n";
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

  if( job_list.empty() ) {
    vw_out() << "Error: Must specify at least two input files!\n\n";
    vw_out() << usage.str();
    return 1;
  }

  ThreadedMatcher matcher( number_threads, fs::path( job_list ).stem()+".match.gz" );

  std::ifstream job_list_file( job_list.c_str() );
  if ( !job_list_file.is_open() )
    vw_throw(ArgumentErr() << "Unable to open: " << job_list << "\n" );
  std::string left, right;
  job_list_file >> left >> right;
  while ( !job_list_file.eof() ) {
    matcher.add_match( left, right, matcher_threshold, inlier_threshold );

    job_list_file >> left >> right;
  }
  job_list_file.close();

  matcher.process_matches();

  std::cout << "Finished\n";
}
