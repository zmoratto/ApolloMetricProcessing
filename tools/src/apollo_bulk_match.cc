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
#include <vw/InterestPoint.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math.h>
#include "ransac.h"
#include <ANN/ANN.h>

using namespace vw;
using namespace vw::ip;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

int main(int argc, char** argv) {
  std::string job_list;
  double matcher_threshold;
  int inlier_threshold = 20;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message")
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

  std::ifstream job_list_file( job_list.c_str() );
  if ( !job_list_file.is_open() )
    vw_throw(ArgumentErr() << "Unable to open: " << job_list << "\n" );
  while ( !job_list_file.eof() ) {
    std::string left, right;
    job_list_file >> left >> right;
    std::cout << "Finding matches between " << left << " and " << right << "\n";
  }
  job_list_file.close();

}
