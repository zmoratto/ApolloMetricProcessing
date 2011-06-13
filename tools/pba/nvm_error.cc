#include <vw/Core.h>
#include <vw/Math.h>
using namespace vw;

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include "../pba/nvmio.h"

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
  usage << "Usage: " << argv[0] << " [options] <nvm> ...\n";

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

    std::cout << "Loading \"" << opt.nvm_input << "\"\n";
    ba::ControlNetwork cnet("nvm_error");
    read_nvm_iterator_ptr( opt.nvm_input,
                           opt.cameras, cnet );
    std::cout << "Found cameras: " << opt.cameras.size() << "\n";
    std::cout << "Found points : " << cnet.size() << "\n";

    double error_accumulated = 0;
    uint64 error_cnt = 0;
    math::CDFAccumulator<double> error_cdf;

    float inc_amt = 1.0/float(cnet.size());
    TerminalProgressCallback tpc("","Err: ");
    tpc.report_progress(0);
    BOOST_FOREACH( ba::ControlPoint& cp, cnet ) {
      BOOST_FOREACH( ba::ControlMeasure& cm, cp ) {
        double error =
          norm_2(cm.position() - opt.cameras[cm.image_id()]->point_to_pixel(cp.position()));
        error_cdf(error);
        error_cnt++;
        error_accumulated+=error;
      }
      tpc.report_incremental_progress(inc_amt);
    }
    tpc.report_finished();
    error_cdf.update();

    std::cout << "Before--------------------------------\n";
    std::cout << "  Error sum:  " << error_accumulated << "\n";
    std::cout << "  Error mean: " << error_accumulated / double(error_cnt) << "\n";
    std::cout << "  Apprx mean: " << error_cdf.approximate_mean(0.02) << "\n";
    std::cout << "  [" << error_cdf.quantile(0) << " "
              << error_cdf.quantile(.25) << " "
              << error_cdf.quantile(.5)  << " "
              << error_cdf.quantile(.75) << " "
              << error_cdf.quantile(1)   << "]\n";

    tpc.set_progress_text("Cut: ");
    error_accumulated = 0;
    error_cnt = 0;
    error_cdf = math::CDFAccumulator<double>();
    size_t cpi = 0;
    tpc.report_progress(0);
    while ( cpi < cnet.size() ) {
      size_t cmi = 0;
      while ( cmi < cnet[cpi].size() ) {
        double error =
          norm_2(cnet[cpi][cmi].position() - opt.cameras[cnet[cpi][cmi].image_id()]->point_to_pixel(cnet[cpi].position()));
        if ( error > 100 ) {
          cnet[cpi].delete_measure(cmi);
        } else {
          error_cdf(error);
          error_cnt++;
          error_accumulated+=error;
          cmi++;
        }
      }
      if ( cnet[cpi].size() < 2 )
        cnet.delete_control_point(cpi);
      else
        cpi++;
      tpc.report_incremental_progress(inc_amt);
    }
    tpc.report_finished();

    std::cout << "After--------------------------------\n";
    std::cout << "  Error sum:  " << error_accumulated << "\n";
    std::cout << "  Error mean: " << error_accumulated / double(error_cnt) << "\n";
    std::cout << "  Apprx mean: " << error_cdf.approximate_mean(0.02) << "\n";
    std::cout << "  [" << error_cdf.quantile(0) << " "
              << error_cdf.quantile(.25) << " "
              << error_cdf.quantile(.5)  << " "
              << error_cdf.quantile(.75) << " "
              << error_cdf.quantile(1)   << "]\n";

    write_nvm_iterator_ptr( "cut_" + opt.nvm_input,
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
