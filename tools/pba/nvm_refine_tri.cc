#include <vw/Core.h>
#include <vw/Math.h>
using namespace vw;

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include "../pba/nvmio.h"

struct PointSolveModel : public math::LeastSquaresModelBase<PointSolveModel> {
  typedef Vector<double> result_type;
  typedef Vector3 domain_type;
  typedef Matrix<double> jacobian_type;

  std::vector<boost::shared_ptr<camera::PinholeModel> > const& cameras;
  ba::ControlPoint const& control_point;
  PointSolveModel(std::vector<boost::shared_ptr<camera::PinholeModel> > const& c,
                  ba::ControlPoint const& p) :
    cameras(c), control_point(p) {}

  inline result_type operator()( domain_type const& x ) const {
    Vector<double> image_e;
    image_e.set_size( control_point.size() * 2 );
    for ( size_t i = 0; i < control_point.size(); i++ ) {
      subvector( image_e, i*2, 2 ) =
        control_point[i].position() -
        cameras[control_point[i].image_id()]->point_to_pixel(x);
    }
    return image_e;
  }
};

struct Options {
  std::string nvm_input;
  std::vector<boost::shared_ptr<camera::PinholeModel> > cameras;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.nvm_input));

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <singleton nvm> ...\n";

  po::variables_map vm;
  try {
    po::options_description all_options;
    all_options.add(general_options).add(positional);
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error const& e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n"
              << e.what() << "\n" << usage.str() << "\n" << general_options );
  }

  if ( opt.nvm_input.empty() )
    vw_throw( ArgumentErr() << "Missing input nvm files!\n"
              << usage.str() << general_options );
  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << "\n" << general_options );
}

int main( int argc, char* argv[] ) {
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // This does a better job of triangulating the positions in the nvm
    ba::ControlNetwork cnet("nvm_refine_tri");
    read_nvm_iterator_ptr( opt.nvm_input,
                           opt.cameras, cnet );
    std::cout << "Found cameras: " << opt.cameras.size() << "\n";
    std::cout << "Found points:  " << cnet.size() << "\n";

    float inc_amt = 1.0/float(cnet.size());
    TerminalProgressCallback tpc("","Cnet: ");
    tpc.report_progress(0);
    BOOST_FOREACH( ba::ControlPoint& p, cnet ) {
      PointSolveModel model( opt.cameras, p );
      int status;
      Vector<double> observation;
      observation.set_size(2*p.size());
      Vector3 solution =
        math::levenberg_marquardt( model, p.position(),
                                   observation, status );
      p.set_position(solution);
      tpc.report_incremental_progress(inc_amt);
    }
    tpc.report_finished();

    // Write control point
    write_nvm_iterator_ptr( "refine_" + opt.nvm_input,
                            opt.cameras.begin(), opt.cameras.end(),
                            cnet );
  } catch( Exception const& e ) {
    std::cerr << "\n\nVW Error: " << e.what() << std::endl;
    return 1;
  } catch( std::exception const& e ) {
    std::cerr << "\n\nVW Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
