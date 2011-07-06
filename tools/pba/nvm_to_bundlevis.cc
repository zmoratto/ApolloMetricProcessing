#include <vw/Core.h>
#include <vw/Math.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;

int main(int argc, char** argv) {
  std::string nvm_file;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("nvm-file", po::value(&nvm_file));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("nvm-file", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <nvm> ... \n\n";
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser(argc,argv).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_out() << "An error occured while parsing command line arguments.\n";
    vw_out() << "\t" << e.what() << "\n\n";
    vw_out() << usage.str();
    return 1;
  }

  if(vm.count("help") ) {
    vw_out() << usage.str();
    return 1;
  } else if ( nvm_file.empty() ) {
    vw_out() << "Missing input nvm file!\n";
    vw_out() << usage.str() << "\n";
    return 1;
  }

  // Output file
  std::ofstream ocam_file("cam.txt", std::ios::out );
  std::ofstream opts_file("pts.txt", std::ios::out );
  ocam_file << std::setprecision(12);
  opts_file << std::setprecision(12);

  // Load NVM
  std::ifstream file(nvm_file.c_str(), std::ios::in);
  if ( !file.is_open() )
    vw_throw( ArgumentErr() << "Unable to open: " << nvm_file << "!\n" );

  std::string key;
  size_t num_cameras, num_points;
  file >> key >> num_cameras;
  std::cout << "Key: " << key << "\n";
  std::cout << "Num Cams: " << num_cameras << "\n";

  // Reading out camera
  for ( size_t j = 0; j < num_cameras; j++ ) {
    std::string name;
    float focal;
    Matrix3x3 rotation;
    Vector3 translation;
    int buf;

    file >> name >> focal >> rotation[0][0] >> rotation[0][1] >> rotation[0][2]
         >> rotation[1][0] >> rotation[1][1] >> rotation[1][2]
         >> rotation[2][0] >> rotation[2][1] >> rotation[2][2]
         >> translation[0] >> translation[1] >> translation[2]
         >> buf >> buf;

    Quat q(transpose(rotation));
    translation = -transpose(rotation)*translation;
    ocam_file << j << "\t" << translation[0] << "\t"
              << translation[1] << "\t" << translation[2] << "\t"
              << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[3] << "\n";
  }

  file >> num_points;
  std::cout << "Num Pts: " << num_points << "\n";

  // Reading out points
  for ( size_t i = 0; i < num_points; i++ ) {
    Vector3 point;
    file >> point[0] >> point[1] >> point[2];

    // Advance file as I don't care for the rest of the information
    char c = file.get();
    while ( c != '\n' && file.good() )
      c = file.get();

    opts_file << i << "\t" << point[0] << "\t" << point[1] << "\t" << point[2] << "\n";
  }

  // Closing up
  ocam_file.close();
  opts_file.close();
}
