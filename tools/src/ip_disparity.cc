#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include "Kriging.h"
#include <vw/Stereo/DisparityMap.h>

using namespace vw;
using namespace vw::ip;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

int main(int argc, char** argv) {
  std::string left, right;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("left-image", po::value(&left))
    ("right-image", po::value(&right));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("left-image", 1);
  p.add("right-image", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <filenames>...\n\n";
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_out() << "An error occured while parsing command line arguments.\n";
    vw_out() << "\t" << e.what() << "\n\n";
    vw_out() << usage.str();
    return 1;
  }

  if( vm.count("help") ) {
    vw_out() << usage.str();
    return 1;
  }

  if ( left.empty() || right.empty() ) {
    vw_out() << "Error: Must specify at least two input files!" << std::endl << std::endl;
    vw_out() << usage.str();
    return 1;
  }

  // Loading interest points
  typedef std::vector< ip::InterestPoint > IPVector;
  IPVector left_ip, right_ip;
  std::string prefix =
    fs::path(left).replace_extension("").string() + "__" +
    fs::path(right).replace_extension("").string();
  std::string output_filename = prefix + ".match";
  read_binary_match_file( output_filename, left_ip, right_ip );

  // Producing the KrigingView
  typedef std::pair<Vector2f, Vector2f> pair_list;
  std::list<pair_list > samples;
  for ( size_t i = 0; i < left_ip.size(); i++ ) {
    Vector2f lloc( left_ip[i].x, left_ip[i].y ),
      rloc( right_ip[i].x, right_ip[i].y );
    samples.push_back( pair_list(lloc,rloc-lloc) );
  }
  DiskImageView<PixelGray<uint8> > image( right );
  KrigingView<Vector2f> disparity( samples,
                                   BBox2i(0,0,image.cols(),image.rows()) );
  ImageViewRef<PixelMask<Vector2f> > pdisparity =
    pixel_cast<PixelMask<Vector2f> >( disparity );
  ImageViewRef<PixelGray<uint8> > right_transformed =
    transform( image, stereo::DisparityTransform( pdisparity ) );
  /*
  DiskImageResourceGDAL rsrc( prefix+"-D.tif",
                              pdisparity.format(), 256 );
  block_write_image(rsrc, pdisparity,
                    TerminalProgressCallback("tools","Writing:") );
  */
  write_image( fs::path(right).replace_extension("").string()+"-Trans.tif", right_transformed,
               TerminalProgressCallback("tools","Writing:") );
}
