#include <vw/Cartography.h>
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <asp/IsisIO.h>

using namespace vw;
using namespace vw::camera;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

#include "LolaQuery.h"
#include "ApolloShapes.h"

int main( int argc, char** argv ) {
  std::string cube_file(argv[0]); 
  if ( cube_file.empty() )
    return 1;

  // Loading CONSTANT measurement data
  LOLAQuery lola_database;
  std::string wac_file("/Users/zmoratto/Data/Moon/LROWAC/global_100m_JanFeb_and_JulyAug.180.cub");
  DiskImageView<PixelGray<float> > wac_image( wac_file );
  float wac_nodata_value = -3.40282265508890445e+38;
  std::cout << "WAC nodata: " << wac_nodata_value << "\n";
  cartography::GeoReference wac_georef;
  cartography::read_georeference( wac_georef,  wac_file );
  std::cout << "Using WAC georef:\n" << wac_georef << "\n";
  double wac_degree_scale =
    norm_2(wac_georef.pixel_to_lonlat( Vector2( wac_image.cols(), wac_image.rows() ) / 2 + Vector2(1,0) )
           - wac_georef.pixel_to_lonlat( Vector2( wac_image.cols(), wac_image.rows() ) / 2 ));
  std::cout << "WAC Degree scale: " << wac_degree_scale << "\n";

  // Loading input camera and image
  boost::shared_ptr<camera::CameraModel> model;
  DiskImageView<PixelGray<float> > image( fs::path( cube_file ).replace_extension(".tif").string() );
  std::string adjust_file =
    fs::path( cube_file ).replace_extension("isis_adjust").string();
  std::string serial_number;
  if ( fs::exists( adjust_file ) ) {
    vw_out() << "Loading \"" << adjust_file << "\"\n";
    std::ifstream input( adjust_file.c_str() );
    boost::shared_ptr<asp::BaseEquation> posF = asp::read_equation(input);
    boost::shared_ptr<asp::BaseEquation> poseF = asp::read_equation(input);
    input.close();
    boost::shared_ptr<camera::IsisAdjustCameraModel> child( new camera::IsisAdjustCameraModel( cube_file, posF, poseF ) );
    serial_number = child->serial_number();
    model = child;
  } else {
    vw_out() << "Loading \"" << cube_file << "\"\n";
    boost::shared_ptr<camera::IsisCameraModel> child(new camera::IsisCameraModel( cube_file ) );
    serial_number = child->serial_number();
    model = child;
  }

  // Working out scale and rotation
  cartography::GeoReference georef( cartography::Datum("D_MOON"),
                                    math::identity_matrix<3>() );
  bool working;
  Vector2 l_direction =
    cartography::geospatial_intersect( Vector2(), georef, model,
                                       1, working ) -
    cartography::geospatial_intersect( Vector2(0,image.rows()-1),
                                       georef, model, 1, working );
  if (l_direction[0] < -200)
    l_direction[0] += 360;
  if (l_direction[0] > 200 )
    l_direction[0] -= 360;
  Vector2 r_direction =
    cartography::geospatial_intersect( Vector2(image.cols()-1,0), georef,
                                       model, 1, working ) -
    cartography::geospatial_intersect( Vector2(image.cols(),image.rows()) - Vector2(1,1),
                                       georef, model, 1, working );
  if (r_direction[0] < -200)
    r_direction[0] += 360;
  if (r_direction[0] > 200)
    r_direction[0] -= 360;
  double rotate = M_PI/2 - (atan2(l_direction[1],l_direction[0]) +
                            atan2(r_direction[1],r_direction[0]) )/2;
  double degree_scale =
    (norm_2(l_direction)+norm_2(r_direction)) / ( image.cols() + image.rows() );
    //norm_2(l_direction+r_direction) / norm_2(Vector2(image.rows(),image.cols()));
  std::cout << "Degree scale: " << degree_scale << "\n";

  // Find the WAC center that we need to focus on
  Vector2 wac_deg_origin =
    subvector(cartography::XYZtoLonLatRadFunctor::apply(model->camera_center(Vector2())),0,2);
  Vector2 wac_pix_origin =
    wac_georef.lonlat_to_pixel( wac_deg_origin );

  // Rasterizing a section of WAC at the same scale and area as our
  // image.
  CompositionTransform<TranslateTransform,CompositionTransform<AffineTransform,ResampleTransform> >
    wactx = compose( TranslateTransform( -wac_pix_origin[0], -wac_pix_origin[1] ),
                     RotateTransform( -rotate, Vector2() ),
                     ResampleTransform( degree_scale/wac_degree_scale,
                                        degree_scale/wac_degree_scale) );
  ImageViewRef<PixelGray<float> > wac_temporary =
    apply_mask(normalize(crop(transform(create_mask(wac_image,wac_nodata_value), wactx,
                                        CylindricalEdgeExtension()),
                              -image.cols()/2, -image.rows()/2,
                              image.cols(), image.rows())));
  write_image( "wac_crop.tif", wac_temporary );

  return 0;
}
