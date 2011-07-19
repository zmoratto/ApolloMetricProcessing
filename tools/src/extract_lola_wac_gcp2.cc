/* This is version 2, which attempts to extract data from a much
   higher resolution version of LOLA.
 */
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Core/Debugging.h>

#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/ControlNetTK/Equalization.h>

#include <boost/foreach.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include "LolaQuery.h"

using namespace vw;

//  compute_normals()
//
// Compute a vector normal to the surface of a DEM for each given
// pixel.  The normal is computed by forming a plane with three points
// in the vicinity of the requested pixel, and then finding the vector
// normal to that plane.  The user must specify the scale in the [u,v]
// directions so that the direction of the vector in physical space
// can be properly ascertained.  This is often contained in the (0,0)
// and (1,1) entry of the georeference transform.
class ComputeNormalsFunc : public ReturnFixedType<PixelMask<Vector3f> >
{
  float m_u_scale, m_v_scale;

public:
  ComputeNormalsFunc(float u_scale, float v_scale) :
    m_u_scale(u_scale), m_v_scale(v_scale) {}

  BBox2i work_area() const { return BBox2i(Vector2i(0, 0), Vector2i(1, 1)); }

  template <class PixelAccessorT>
  PixelMask<Vector3f> operator() (PixelAccessorT const& accessor_loc) const {
    PixelAccessorT acc = accessor_loc;

    // Pick out the three altitude values.
    if (is_transparent(*acc))
      return PixelMask<Vector3f>();
    float alt1 = *acc;

    acc.advance(1,0);
    if (is_transparent(*acc))
      return PixelMask<Vector3f>();
    float alt2 = *acc;

    acc.advance(-1,1);
    if (is_transparent(*acc))
      return PixelMask<Vector3f>();
    float alt3 = *acc;

    // Form two orthogonal vectors in the plane containing the three
    // altitude points
    Vector3f n1(m_u_scale, 0, alt2-alt1);
    Vector3f n2(0, m_v_scale, alt3-alt1);

    // Return the vector normal to the local plane.
    return normalize(cross_prod(n1,n2));
  }
};

template <class ViewT>
UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, ComputeNormalsFunc> compute_normals(ImageViewBase<ViewT> const& image,
                                                                                                              float u_scale, float v_scale) {
  return UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, ComputeNormalsFunc>(edge_extend(image.impl(), ConstantEdgeExtension()),
                                                                                                       ComputeNormalsFunc (u_scale, v_scale));
}

class DotProdFunc : public ReturnFixedType<float > {
  Vector3f m_vec;
public:
  DotProdFunc(Vector3f const& vec) : m_vec(vec) {}
  float operator() (PixelMask<Vector3f> const& pix) const {
    if (is_transparent(pix))
      return 0;
    else
      return dot_prod(pix.child(),m_vec)/(norm_2(pix.child()) * norm_2(m_vec));
  }
};

template <class ViewT>
UnaryPerPixelView<ViewT, DotProdFunc> dot_prod(ImageViewBase<ViewT> const& view, Vector3f const& vec) {
  return UnaryPerPixelView<ViewT, DotProdFunc>(view.impl(), DotProdFunc(vec));
}


int main( int argc, char* argv[] ) {
  LOLAQuery database;

  std::string input_cube;
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input", po::value(&input_cube));

  po::options_description options("");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <cube> ... \n\n";
  usage << general_options << std::endl;

  if ( vm.count("help")  || input_cube.empty() ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  }

  // Generate Data
  boost::shared_ptr<camera::CameraModel> model;
  std::string adjust_file =
    fs::path( input_cube ).replace_extension("isis_adjust").string();
  Vector2i size;
  std::string serial;
  Vector3 sun_position;
  Quat camera_pose;
  if ( fs::exists( adjust_file ) ) {
    vw_out() << "Loading \"" << adjust_file << "\"\n";
    std::ifstream input( adjust_file.c_str() );
    boost::shared_ptr<asp::BaseEquation> posF = asp::read_equation(input);
    boost::shared_ptr<asp::BaseEquation> poseF = asp::read_equation(input);
    input.close();
    boost::shared_ptr<camera::IsisAdjustCameraModel> child( new camera::IsisAdjustCameraModel( input_cube, posF, poseF ) );
    camera_pose = child->camera_pose();
    serial = child->serial_number();
    size = Vector2i(child->samples(),child->lines());
    sun_position = child->sun_position();
    model = child;
  } else {
    vw_out() << "Loading \"" << input_cube << "\"\n";
    boost::shared_ptr<camera::IsisCameraModel> child(new camera::IsisCameraModel( input_cube ) );
    camera_pose = child->camera_pose();
    serial = child->serial_number();
    size = Vector2i(child->samples(),child->lines());
    sun_position = child->sun_position();
    model = child;
  }

  // Convert sun_position to camera domain
  sun_position =
    inverse(camera_pose).rotate(sun_position - model->camera_center(Vector2()));
  sun_position = normalize(sun_position);
  sun_position[2] = -sun_position[2];

  // Load data
  std::pair<cartography::GeoReference, std::string> result =
    database.find_tile( cartography::XYZtoLonLatRadFunctor::apply(model->camera_center(Vector2())) );
  std::cout << "Using tile: " << result.second << "\n";
  if ( result.first.transform()(0,2) >= 180 ) {
    Matrix3x3 t = result.first.transform();
    t(0,2) -= 360;
    result.first.set_transform(t);
  }
  std::cout << result.first << "\n";

  // Working out scale and rotation
  float degree_scale;
  BBox2 degree_bbox =
    cartography::camera_bbox( result.first, model, size[0], size[1], degree_scale );
  if ( degree_bbox.min()[0] < 0 ) {
    degree_bbox.max()[0] += 360;
    degree_bbox.min()[0] += 360;
  }

  degree_bbox.expand(2);
  bool working;
  Vector2 l_direction =
    cartography::geospatial_intersect( Vector2(), result.first, model,
                                       1, working ) -
    cartography::geospatial_intersect( Vector2(0,size[1]), result.first, model,
                                       1, working );
  if (l_direction[0] < -200)
    l_direction[0] += 360;
  if (l_direction[0] > 200 )
    l_direction[0] -= 360;
  Vector2 r_direction =
    cartography::geospatial_intersect( Vector2(size[0],0), result.first, model,
                                       1, working ) -
    cartography::geospatial_intersect( size, result.first, model,
                                       1, working );
  if (r_direction[0] < -200)
    r_direction[0] += 360;
  if (r_direction[0] > 200)
    r_direction[0] -= 360;
  double rotate = M_PI/2 - (atan2(l_direction[1],l_direction[0]) +
                            atan2(r_direction[1],r_direction[0]) )/2;
  degree_scale = (norm_2(l_direction)+norm_2(r_direction)) / (size[1]*2);

  // Debug information
  vw_out() << "\tBBox: " << degree_bbox << " deg\n";
  vw_out() << "\tScle: " << degree_scale << "\n"; // Degrees / pixel I assume
  vw_out() << "\t    : " << 1/degree_scale  << " px/deg\n";
  vw_out() << "\tRot : " << rotate*180/M_PI << " deg\n\n";

  // Creating output georef
  cartography::GeoReference georef_out;
  georef_out.set_datum( cartography::Datum("Sphere","Sphere","Zero",
                                           180.0/M_PI, 180.0/M_PI, 0) );
  georef_out.set_orthographic( degree_bbox.min()[1] + degree_bbox.height()/2,
                               degree_bbox.min()[0] + degree_bbox.width()/2 );

  // Transform defines pixel to point
  Matrix3x3 tx = math::identity_matrix<3>();
  tx(0,2) = -size[0]*degree_scale/2;
  tx(1,2) = size[1]*degree_scale/2;
  tx(0,0) = degree_scale;
  tx(1,1) = -degree_scale;
  georef_out.set_transform( tx );
  std::cout << "Georef: " << georef_out << "\n";
  TransformRef wactx( compose( RotateTransform( -rotate,
                                                (Vector2(size)-Vector2(1,1))/2 ),
                               cartography::GeoTransform( result.first, georef_out ) ) );

  double pixel_meters = 2*M_PI*1737400 / 360 * degree_scale;
  std::cout << "Meters per Pixel: " << pixel_meters << "\n";

  cartography::GeoTransform geotx( result.first, georef_out );
  std::cout << "Reverse bbox: " << geotx.reverse_bbox(BBox2i(0,0,size[0],size[1])) << "\n";

  DiskImageView<float> input( result.second );
  ImageViewRef<float> local_dem =
    crop( transform( input, wactx ),
          BBox2i(0,0,size[0],size[1]) );
  ImageViewRef<float> new_lighting =
    gaussian_filter(clamp(dot_prod(compute_normals(local_dem,
                                                   pixel_meters,pixel_meters),
                                   sun_position)),2);
  // Guess the WAC mosaic lighting
  Vector3 wac_light = math::rotation_z_axis(rotate)*Vector3(-3,0,1);
  wac_light = normalize(wac_light);
  std::cout << "Wac Normal: " << wac_light << "\n";
  std::cout << "Sun Normal: " << sun_position << "\n";
  ImageViewRef<float> prv_lighting =
    gaussian_filter(clamp(dot_prod(compute_normals(local_dem,
                                                   pixel_meters, pixel_meters),
                                   wac_light)),2);

  write_image("test.tif", prv_lighting,
              TerminalProgressCallback("tools","Prv Lighting:") );
  write_image("wac_test.tif", new_lighting,
              TerminalProgressCallback("tools","New Lighting:") );

  // Loading wac image
  std::string wac_file =
    fs::path( input_cube ).replace_extension("wac.tif").string();
  DiskImageView<float> wac_image( wac_file );
  write_image("relit.tif",
              normalize(wac_image * new_lighting * (constant_view(1.0f,size[0],size[1])-prv_lighting)),
              TerminalProgressCallback("tools","Lighting: ") );

  return 0;
}
