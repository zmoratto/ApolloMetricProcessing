#include <vw/Core.h>
#include <vw/Camera/PinholeModel.h>
#include <boost/foreach.hpp>
#include <asp/IsisIO.h>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

using namespace vw;
using namespace camera;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main( int argc, char* argv[] ) {
  std::string csv_file;
  std::vector<std::string> input_cubes;
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-csv", po::value(&csv_file))
    ("input-files", po::value(&input_cubes));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-csv", 1);
  p.add("input-files", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <image-files> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( input_cubes.empty() || csv_file.empty() ) {
    vw_out() << "ERROR! Require an input file.\n";
    vw_out() << "\n" << usage.str() << "\n";
    return 1;
  }

  // Load up CSV file
  std::cout << "Reading CSV file\n";
  std::map<std::string,Vector3> corrected_positions;
  std::ifstream csv( csv_file.c_str() );
  std::string line;
  std::getline( csv, line );
  while ( csv.good() ) {
    boost::char_separator<char> sep(", ");
    typedef boost::tokenizer<boost::char_separator<char> >
      tokenizer;
    tokenizer tokens(line, sep);
    int count = 0;
    std::string name;
    Vector3 position;
    for ( tokenizer::iterator tok_it = tokens.begin();
          tok_it != tokens.end(); tok_it++ ) {
      switch (count) {
      case 0:
        name = *tok_it;
        break;
      case 2:
      case 3:
      case 4:
        position[count-2] = boost::lexical_cast<double>(*tok_it); break;
      default:
        break;
      }
      count++;
    }
    corrected_positions[name] = position;
    std::getline( csv, line );
  }
  csv.close();

  // Load up every input cube and create an isis_adjust for them
  BOOST_FOREACH( std::string const& cube, input_cubes ) {
    std::cout << "Writing adjustment for " << cube << "\n";
    vw::camera::IsisCameraModel camera( cube );
    Vector3 objective = corrected_positions[cube];
    Vector3 delta = objective - camera.camera_center(Vector2());
    boost::shared_ptr<asp::BaseEquation> position(new asp::PolyEquation (0));
    (*position)[0] = delta[0];
    (*position)[1] = delta[1];
    (*position)[2] = delta[2];
    boost::shared_ptr<asp::BaseEquation> pose(new asp::PolyEquation(0));
    std::ofstream adjust( fs::path( cube ).replace_extension( "isis_adjust" ).string().c_str() );
    write_equation( adjust, position );
    write_equation( adjust, pose );
    adjust.close();
  }
}
