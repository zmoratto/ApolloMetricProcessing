#include <vw/Core.h>
#include <boost/foreach.hpp>
#include <asp/IsisIO.h>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem/operations.hpp>

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main( int argc, char* argv[] ) {
  std::vector<std::string> input_cubes;
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value(&input_cubes));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <cube files> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  BOOST_FOREACH( std::string const& cube, input_cubes ) {
    std::string adjust_file =
      fs::path(cube).replace_extension(".isis_adjust").string();
    if ( fs::exists(adjust_file) ) {
      std::cout << "Loading adjusted ISIS camera model\n";
    } else {
      std::cout << "Loading plain ISIS camera model\n";
    }
  }
}
