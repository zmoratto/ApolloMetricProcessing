///
/// Unpacks a apollo_bulk_match result file into many match files.
///
#include <vw/InterestPoint.h>

using namespace vw;
using namespace vw::ip;

#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;
#include <boost/foreach.hpp>

int main(int argc, char** argv) {
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

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <bulk_match>...\n\n";
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

  if( input_file_names.empty() ) {
    vw_out() << "Error: Must specify at least two input files!\n\n";
    vw_out() << usage.str();
    return 1;
  }

  BOOST_FOREACH( std::string const& file, input_file_names ) {

    std::cout << "Loading : " << file << "\n";

  }

}
