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

#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file.hpp>
namespace io = boost::iostreams;

#include "iprecord.h"

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
    io::filtering_istream in;
    in.push( io::gzip_decompressor() );
    in.push( io::file_source( file ) );

    { // Check for magic word
      int magic_size;
      std::string magic;
      in.read( (char*)&magic_size, sizeof(magic_size) );
      magic.resize( magic_size );
      in.read( &magic[0], magic_size );
      std::cout << "Magic: " << magic << "\n";
      if ( magic != "Packed Match File!" )
        vw_throw( IOErr() << "Seem to have wrong or corrupt packed match file!\n" );
    }

    char name[256];
    int header_size;
    std::string header;
    in.read( (char*)&header_size, sizeof(header_size) );
    while ( in.good() ) {
      std::cout << "Header size: " << header_size << "\n";
      header.resize( header_size );
      in.read( &header[0], header_size );
      std::cout << "Found: " << header << "\n";

      std::vector<InterestPoint> ip1, ip2;
      int match_size;
      in.read( (char*)&match_size, sizeof(match_size) );
      for ( unsigned i = 0; i < match_size; i++ ) {
        ip1.push_back( read_ip_record( in ) );
      }
      in.read( (char*)&match_size, sizeof(match_size) );
      for ( unsigned i = 0; i < match_size; i++ ) {
        ip2.push_back( read_ip_record( in ) );
      }

      in.read( (char*)&header_size, sizeof(header_size) );
    }
  }

}
