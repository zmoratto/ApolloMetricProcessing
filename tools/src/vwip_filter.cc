// Zack Moratto
// Meant to be used with cube files with no other information.
// *WARNING* This has been modifed from the original used on the super computer
// to use only VWIP stuff.

// std header
#include <stdlib.h>
#include <iostream>
#include <vector>

// Boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

// Vision Workbench
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/Matcher.h>
using namespace vw;
using namespace vw::ip;

// Dilate
void dilate( ImageView<PixelGray<uint8> > const& im ) {
  for ( int i = 0; i < im.cols(); i++ ) {
    for ( int j = 0; j < im.rows(); j++ ) {
      if ( im(i,j) == 255 ) {
        if ( i > 0 && im(i-1,j) == 0 )
          im(i-1,j) = 2;
        if ( j > 0 && im(i,j-1) == 0 )
          im(i,j-1) = 2;
        if ( i+1 < im.cols() && im(i+1,j) == 0 )
          im(i+1,j) = 2;
        if ( j+1 < im.rows() && im(i,j+1) == 0 )
          im(i,j+1) = 2;
      }
    }
  }
  for ( int i = 0; i < im.cols(); i++ ) {
    for ( int j = 0; j < im.rows(); j++ ) {
      if ( im(i,j) == 2 )
        im(i,j) = 255;
    }
  }
}

// Erode
void erode( ImageView<PixelGray<uint8> > const& im ) {
  for ( int i = 0; i < im.cols(); i++ ) {
    for ( int j = 0; j < im.rows(); j++ ) {
      if ( im(i,j) == 0 ) {
        if ( i > 0 && im(i-1,j) == 255 )
          im(i-1,j) = 2;
        if ( j > 0 && im(i,j-1) == 255 )
          im(i,j-1) = 2;
        if ( i+1 < im.cols() && im(i+1,j) == 255 )
          im(i+1,j) = 2;
        if ( j+1 < im.rows() && im(i,j+1) == 255 )
          im(i,j+1) = 2;
      }
    }
  }
  for ( int i = 0; i < im.cols(); i++ ) {
    for ( int j = 0; j < im.rows(); j++ ) {
      if ( im(i,j) == 2 )
        im(i,j) = 0;
    }
  }
}

// Thresholding Func
class ThresholdFunc : public vw::UnaryReturnSameType {

  uint8 m_top, m_bot;

public:
  ThresholdFunc( uint8 const& bot, uint8 const& top ) : m_top(top), m_bot(bot) {}

  uint8 operator() (uint8 const& px) const {
    //std::cout << int(px);
    if ( px < m_bot || px > m_top )
      return 255;
    return 0;
  }

};

// Main
int main(int argc, char* argv[]) {

  std::string tif_file, vwip_file;

  // Boost Program Options code
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("image",po::value<std::string>(&tif_file), "image")
    ("vwip",po::value<std::string>(&vwip_file), "vwip");

  po::positional_options_description positional_options_desc;
  positional_options_desc.add("image", 1);
  positional_options_desc.add("vwip",1);

  po::options_description all_options;
  all_options.add(general_options).add(positional_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_options_desc).run(), vm );
  po::notify(vm);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <tif image> <vwip> [options] ... " << std::endl;
  usage << general_options << std::endl;

  if ( vm.count("help")) {
    std::cout << usage.str() << std::endl;
    exit(0);
  }

  if ( !vm.count("image") || !vm.count("vwip") ) {
    std::cout << "\n Missing input files.\n";
    std::cout << usage.str() << std::endl;
    exit(0);
  }

  // Loading IP files
  std::vector<InterestPoint> l_ip = read_binary_ip_file( vwip_file );

  // Load Image and Render Binary
  DiskImageView<PixelGray<uint8> > input( tif_file );
  ImageView<PixelGray<uint8> > binary = per_pixel_filter( input, ThresholdFunc(20,235) );

  for ( uint i = 0; i < 3; i++ )
    erode(binary);
  for ( uint i = 0; i < 13; i++ )
    dilate(binary);

  // Filter interest points
  std::vector<InterestPoint>::iterator pos = l_ip.begin();
  while ( pos != l_ip.end() ) {
    if ( binary(pos->ix,pos->iy) == 255 )
      pos = l_ip.erase(pos);
    else
      pos++;
  }

  // Converting to list
  std::list<InterestPoint> list;
  for ( uint i = 0; i < l_ip.size(); i++  )
    list.push_back(l_ip[i]);
  write_binary_ip_file(vwip_file, list);
}
