#include <vw/InterestPoint.h>
#include <vw/Math.h>
using namespace vw;
using namespace ip;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main( int argc, char** argv ) {
  std::string match_file;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("match-file", po::value(&match_file));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("match-file", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <filenames>...\n\n";
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

  if ( vm.count("help") ) {
    vw_out() << usage.str();
    return 1;
  }

  if ( match_file.empty() ) {
    vw_out() << "Error : missing input match file!\n";
    return 1;
  }

  typedef std::vector<InterestPoint> IPVector;
  IPVector ip1, ip2;
  ip::read_binary_match_file( match_file, ip1, ip2 );

  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(ip2);

  typedef math::HomographyFittingFunctor hfit_func;
  typedef math::AffineFittingFunctor afit_func;
  Matrix<double> affine = afit_func()(ransac_ip1,ransac_ip2);
  Matrix<double> homogrphy = hfit_func()(ransac_ip1,ransac_ip2,affine);
  std::cout << "M: " << homogrphy << "\n";
  return 0;
}
