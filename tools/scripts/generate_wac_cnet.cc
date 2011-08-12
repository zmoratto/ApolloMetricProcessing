// This is the follow up application to generate_wac_crop

#include <vw/Cartography.h>
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <asp/IsisIO.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

using namespace vw;
using namespace vw::camera;

#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

#include "LolaQuery.h"
#include "ApolloShapes.h"

int main( int argc, char** argv ) {
  std::string cube_file(argv[1]);
  if ( cube_file.empty() )
    return 1;

  // Loading CONSTANT measurement data
  LOLAQuery lola_database;
  std::string wac_file("/Users/zmoratto/Data/Moon/LROWAC/global_100m_JanFeb_and_JulyAug.180.cub");
  if (!fs::exists(wac_file))
    wac_file = "/Users/zack/Data/Moon/LROWAC/global_100m_JanFeb_and_JulyAug.180.cub";
  DiskImageView<PixelGray<float> > wac_image( wac_file );
  float wac_nodata_value = -3.40282265508890445e+38;
  cartography::GeoReference wac_georef;
  cartography::read_georeference( wac_georef,  wac_file );
  double wac_degree_scale =
    norm_2(wac_georef.pixel_to_lonlat( Vector2( wac_image.cols(), wac_image.rows() ) / 2 + Vector2(1,0) )
           - wac_georef.pixel_to_lonlat( Vector2( wac_image.cols(), wac_image.rows() ) / 2 ));

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

  // Find the WAC center that we need to focus on
  Vector2 wac_deg_origin =
    subvector(cartography::XYZtoLonLatRadFunctor::apply(model->camera_center(Vector2())),0,2);
  Vector2 wac_pix_origin =
    wac_georef.lonlat_to_pixel( wac_deg_origin );

  // Rasterizing a section of WAC at the same scale and area as our
  // image.
  CompositionTransform<ResampleTransform,CompositionTransform<AffineTransform,TranslateTransform> >
    wactx = compose( ResampleTransform( wac_degree_scale/degree_scale,
                                        wac_degree_scale/degree_scale),
                     RotateTransform( -rotate, Vector2() ),
                     TranslateTransform( -wac_pix_origin[0], -wac_pix_origin[1] ) );

  // Load up measurements
  std::vector<ip::InterestPoint> cube_meas, wac_meas;
  read_binary_match_file("wac.match",cube_meas,wac_meas);

  // Building Control Network
  ba::ControlNetwork cnet("WAC LOLA GCPs v3",ba::ControlNetwork::ImageToGround);
  InterpolationView<EdgeExtensionView<ImageViewRef<double>, ConstantEdgeExtension>, BicubicInterpolation> current_lola_image =
    interpolate(ImageViewRef<double>(), BicubicInterpolation(), ConstantEdgeExtension());
  std::string current_lola_file;
  cartography::GeoReference current_lola_georef;
  for ( std::vector<ip::InterestPoint>::iterator cube_pt = cube_meas.begin(),
          wac_pt = wac_meas.begin(); cube_pt != cube_meas.end(); ++cube_pt, ++wac_pt ) {

    // Convert WAC to actual lonlat location
    Vector2 wac_lonlat =
      wac_georef.pixel_to_lonlat( wactx.reverse( Vector2(wac_pt->x, wac_pt->y) ) );

    // Load corresponding LOLA data
    std::pair<cartography::GeoReference, std::string> result =
      lola_database.find_tile( wac_lonlat );
    if ( result.second != current_lola_file ) {
      current_lola_file = result.second;
      current_lola_georef = result.first;
      current_lola_image =
        interpolate(ImageViewRef<double>(DiskImageView<double>(result.second)),
                    BicubicInterpolation(), ConstantEdgeExtension());
    }
    Vector2 lola_pt = current_lola_georef.lonlat_to_pixel(wac_lonlat);
    double radius = current_lola_image(lola_pt[0],lola_pt[1]) +
      current_lola_georef.datum().radius( wac_lonlat[0], wac_lonlat[1] );

    std::cout << cube_pt->x << " " << cube_pt->y << " -> " << wac_lonlat << " " << radius << "\n";

    // Create the control point and measurement
    ba::ControlPoint cpoint(ba::ControlPoint::GroundControlPoint);
    cpoint.set_position(cartography::LonLatRadToXYZFunctor()(Vector3(wac_lonlat[0],
                                                                     wac_lonlat[1],
                                                                     radius)));
    double sigma = 120 * cube_pt->scale;
    cpoint.set_sigma(Vector3(sigma,sigma,sigma));
    ba::ControlMeasure cm( cube_pt->x, cube_pt->y, 1, 1,
                           ba::ControlMeasure::Automatic );
    cm.set_serial( serial_number );
    cm.set_image_id( 1 );
    cpoint.add_measure( cm );
    cnet.add_control_point( cpoint );
  }

  cnet.write_binary(fs::path(cube_file).stem()+"_lola_wac.cnet");

  return 0;
}
