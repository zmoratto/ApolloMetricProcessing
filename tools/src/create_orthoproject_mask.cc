#include <vw/Image.h>
#include <vw/FileIO.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace vw;

int main( int argc, char* argv[] ) {

  std::string mask_file, rd_file, f_file;
  po::options_description general_options("Options");
  general_options.add_options()
    ("debug", "Write out debug images")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("mask-file", po::value(&mask_file))
    ("rd-disparity", po::value(&rd_file))
    ("f-disparity", po::value(&f_file));

  po::options_description options("");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("mask-file", 1);
  p.add("rd-disparity", 1);
  p.add("f-disparity", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <lmask> <f-disparity> <f-dust-disparity> ...\n\n";
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch ( const po::error& e ) {
    vw_throw(ArgumentErr() << "Could not parse commmand line arguments: " << usage.str());
  }

  if ( vm.count("help") || mask_file.empty() || rd_file.empty() || f_file.empty() ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  }

  // I'm not entirely sure how this is going to work .. however first
  // I'm just going to try an isolate the newly removed holes from
  // F-disp. I'll just diff RD and F.
  ImageViewRef<uint8> holes = select_channel(DiskImageView<PixelRGB<uint8> >(rd_file),2) -
    select_channel(DiskImageView<PixelRGB<uint8> >(f_file), 2);
  if (vm.count("debug") )
    write_image("holes.tif", holes,
                TerminalProgressCallback("", "Writing Holes.tif:") );

  std::string output_name = mask_file.substr(0,mask_file.rfind('-')) + "-OrthoMask.tif";
  write_image( output_name, threshold(gaussian_filter(DiskImageView<uint8>(mask_file) - holes,2),210,255),
               TerminalProgressCallback("", output_name) );

  return 0;
}
