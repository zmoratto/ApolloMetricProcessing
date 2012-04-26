#include <vw/Image.h>
#include <vw/FileIO.h>
#include "ApolloShapes.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace vw;

template <class ViewT>
class ApolloMask : public ImageViewBase<ApolloMask<ViewT> > {
  ViewT m_view;
  std::string m_cube_name;
  int m_image_number;
  ApolloShapes m_shapes;
  double m_scaling;
public:
  typedef typename ViewT::pixel_type pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<ApolloMask<ViewT> > pixel_accessor;

  ApolloMask( ImageViewBase<ViewT> const& view,
              std::string const& cube_name ) : m_view(view.impl()),
                                               m_cube_name(cube_name),
                                               m_image_number(extract_camera_number(cube_name)) {
    m_scaling = 1145. / float(m_view.cols());
  }

  inline int32 cols() const { return m_view.cols(); }
  inline int32 rows() const { return m_view.rows(); }
  inline int32 planes() const { return m_view.planes(); }
  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( int32 i, int32 j, int32 p=0 ) const {
    Vector2f point_loc( m_scaling * i, m_scaling * j );
    if ( m_shapes.in_fiducial( point_loc ) )
      return result_type();
    if ( (m_image_number >= 151941 && m_image_number <= 151944) ||
         (m_image_number >= 152093 && m_image_number <= 152097) ||
         (m_image_number >= 170233 && m_image_number <= 170313) ||
         (m_image_number >= 171981 && m_image_number <= 172124) ) {
      if ( m_shapes.in_lens_cap( point_loc ) )
        return result_type();
    }
    if ( (m_image_number >= 152093 && m_image_number <= 152204) ||
         (m_image_number >= 161145 && m_image_number <= 161650) ||
         (m_image_number >= 161896 && m_image_number <= 161985) ||
         (m_image_number >= 162155 && m_image_number <= 162845) ) {
      if ( m_shapes.in_antenna( point_loc ) )
        return result_type();
    }
    return m_view(i,j,p);
  }

  /// \cond INTERNAL
  typedef ApolloMask<typename ViewT::prerasterize_type> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    return ApolloMask<typename ViewT::prerasterize_type>( m_view.prerasterize(bbox), m_cube_name ); }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }
  /// \endcond
};

template <class ViewT>
ApolloMask<ViewT> apollo_mask( ImageViewBase<ViewT> const& v,
                               std::string const& mask_view ) {
  return ApolloMask<ViewT>( v.impl(), mask_view );
}

int main( int argc, char* argv[] ) {

  std::string mask_file, rd_file, f_file, cube_name;
  po::options_description general_options("Options");
  general_options.add_options()
    ("debug", "Write out debug images")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("mask-file", po::value(&mask_file))
    ("rd-disparity", po::value(&rd_file))
    ("f-disparity", po::value(&f_file))
    ("cube-name", po::value(&cube_name));

  po::options_description options("");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("mask-file", 1);
  p.add("rd-disparity", 1);
  p.add("f-disparity", 1);
  p.add("cube-name", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <lmask> <f-disparity> <f-dust-disparity> <cube-name> ...\n\n";
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch ( const po::error& e ) {
    vw_throw(ArgumentErr() << "Could not parse commmand line arguments: " << usage.str());
  }

  if ( vm.count("help") || mask_file.empty() || rd_file.empty() || f_file.empty() || cube_name.empty() ) {
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
  write_image( output_name,
               apollo_mask(threshold(gaussian_filter(DiskImageView<uint8>(mask_file) - holes,2),210,255),
                           cube_name),
               TerminalProgressCallback("", output_name) );

  return 0;
}
