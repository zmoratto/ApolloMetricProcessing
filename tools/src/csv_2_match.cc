#include <vw/Core.h>
#include <vw/InterestPoint/InterestData.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main( int argc, char* argv[] ) {
  std::string csv_file;
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-csv", po::value(&csv_file));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-csv", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <csv file> " << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( csv_file.empty() ) {
    vw_out() << "ERROR! Require an input file.\n";
    vw_out() << "\n" << usage.str() << "\n";
    return 1;
  }

  // Create data structure to fill
  std::vector<ip::InterestPoint> ip1, ip2;

  // Load up CSV file
  std::cout << "Reading CSV file\n";
  std::ifstream csv( csv_file.c_str() );
  std::string line;
  std::getline( csv, line );
  while ( csv.good() ) {
    boost::char_separator<char> sep(", ");
    typedef boost::tokenizer<boost::char_separator<char> >
      tokenizer;
    tokenizer tokens(line, sep);

    float buffer[4];
    char count = 0;
    for ( tokenizer::const_iterator i = tokens.begin();
          i != tokens.end(); i++ ) {
      buffer[count] = boost::lexical_cast<float>(*i);
      count++;
    }

    ip1.push_back(ip::InterestPoint(buffer[0],buffer[1]));
    ip2.push_back(ip::InterestPoint(buffer[2],buffer[3]));
    std::getline( csv, line );
  }
  csv.close();

  ip::write_binary_match_file(fs::path(csv_file).replace_extension().string()+".match",ip1,ip2);
}
