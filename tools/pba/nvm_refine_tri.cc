#include <vw/Core.h>
#include <vw/Math.h>
using namespace vw;

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main( int argc, char* argv[] ) {
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // This does a better job of triangulating the positions in the nvm

  } catch( Exception const& e ) {
    std::cerr << "\n\nVW Error: " << e.what() << std::endl;
    return 1;
  } catch( std::exception const& e ) {
    std::cerr << "\n\nVW Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
