#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include "LolaQuery.h"
#include "ApolloShapes.h"

#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/ControlNetTK/Equalization.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;

int main( int argc, char* argv[] ) {

  std::string cube_file;
  po::options_description general_options("Options");
  general_options.add_options()
    ("cube-file", po::value(&cube_file), "Input cube file.")
    ("help,h", "Display this help message");

  po::positional_options_description p;
  p.add("cube-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(general_options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <cube file>\n\n";
  usage << general_options << std::endl;

  if ( vm.count("help" ) ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( cube_file.empty() ) {
    vw_out() << "ERROR! Missing input file.\n";
    vw_out() << usage.str() << std::endl;
    return 1;
  }

  // Loading CONSTANT measurement data
  LOLAQuery lola_database;
  std::string wac_file("/Users/zmoratto/Data/Moon/LROWAC/global_100m_JanFeb_and_JulyAug.180.cub");
  DiskImageView<PixelGray<float> > wac_image( wac_file );
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
  Vector2 pivot = (Vector2(image.cols(),image.rows()) - Vector2(1,1))/2;

  Vector2 extrema1 =
    RotateTransform( rotate, pivot ).forward( Vector2(image.cols(),image.rows()) - Vector2(1,1) ) -
    Vector2(image.cols(),image.rows()) + Vector2(1,1);
  Vector2 extrema2 =
    RotateTransform( rotate, pivot ).forward( Vector2() ) -
    Vector2(image.cols(),image.rows()) + Vector2(1,1);
  double extension =
    std::max( max( extrema1 ), max( extrema2 ) );
  CompositionTransform<TranslateTransform,RotateTransform>
    trans( TranslateTransform( extension, extension ),
           RotateTransform( rotate, pivot ) );
  Vector2 trans_image_size(extension*2+image.cols(),
                           extension*2+image.rows());
  DiskCacheImageView<PixelGray<float> > trans_cache(
      crop(transform( image, trans ), 0, 0,
           trans_image_size[0], trans_image_size[1]), "tif",
      TerminalProgressCallback("","Caching Input:") );

  // Rasterizing a section of WAC at the same scale and area as our
  Vector2 wac_deg_origin =
    subvector(cartography::XYZtoLonLatRadFunctor::apply(model->camera_center(Vector2())),0,2) -
    elem_prod(Vector2(1,-1),(trans_image_size * degree_scale)/2);
  Vector2 wac_pix_origin =
    wac_georef.lonlat_to_pixel( wac_deg_origin );
  CompositionTransform<ResampleTransform,TranslateTransform>
    wac_trans( ResampleTransform( wac_degree_scale/degree_scale,
                                  wac_degree_scale/degree_scale),
               TranslateTransform( -wac_pix_origin[0], -wac_pix_origin[1] ) );
  DiskCacheImageView<PixelGray<float> > wac_cache(
      normalize(crop(transform( wac_image, wac_trans,
                                CylindricalEdgeExtension()), 0, 0,
                     trans_image_size[0], trans_image_size[1])), "tif",
      TerminalProgressCallback("","Caching WAC:") );

  // OBALOG WAC and Input
  ip::InterestPointList trans_ip, wac_ip;

  { // Input IP
    ip::OBALoGInterestOperator interest_operator(.035);
    ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator> detector( interest_operator,
                                                                            0 );
    ip::SGradDescriptorGenerator gen;

    trans_ip = detect_interest_points(trans_cache,detector);
    ApolloShapes shapes;
    {
      BBox2f trans_bbox = bounding_box(trans_cache);
      trans_bbox.contract(1);
      int32 image_number = extract_camera_number( cube_file );
      for ( ip::InterestPointList::iterator point = trans_ip.begin();
            point != trans_ip.end(); ++point ) {
        Vector2f point_loc = trans.reverse( Vector2f(point->x,
                                                     point->y) );
        bool todelete = false;
        if ( !trans_bbox.contains( point_loc ) ||
             shapes.in_fiducial( point_loc ) )
          todelete = true;
        if ( image_number == 151944 ||
             (image_number >= 170233 && image_number <= 170313) ||
             (image_number >= 171981 && image_number <= 172124) ) {
          if ( shapes.in_lens_cap( point_loc ) )
            todelete = true;
        }
        if ( (image_number >= 152093 && image_number <= 152204) ||
             (image_number >= 161145 && image_number <= 161650) ||
             (image_number >= 161897 && image_number <= 161985) ||
             (image_number >= 162156 && image_number <= 162845) ) {
          if ( shapes.in_antenna( point_loc ) )
            todelete = true;
        }
        if ( todelete ) {
          point = trans_ip.erase(point);
          point--;
          continue;
        }
      }
    }
    vw_out() << "\tDetected Input IP: " << trans_ip.size() << "\n";
    BOOST_FOREACH( ip::InterestPoint& pt, trans_ip ) {
      float scaling = 1.0f / pt.scale;
      ImageView<PixelGray<float> > patch =
        transform( trans_cache,
                   AffineTransform( Matrix2x2( scaling, 0, 0, scaling ),
                                    Vector2(-scaling*pt.x+20.5,
                                            -scaling*pt.y+20.5) ),
                   42, 42 );
      pt.descriptor.set_size( 180 );
      gen.compute_descriptor( patch, pt.begin(), pt.end() );
    }
  }

  { // WAC IP
    ip::OBALoGInterestOperator interest_operator(.020);
    ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator> detector( interest_operator,
                                                                            0 );
    ip::SGradDescriptorGenerator gen;

    wac_ip = detect_interest_points(wac_cache,detector);
    vw_out() << "\tDetected WAC IP: " << wac_ip.size() << "\n";
    BOOST_FOREACH( ip::InterestPoint& pt, wac_ip ) {
      float scaling = 1.0f / pt.scale;
      ImageView<PixelGray<float> > patch =
        transform( wac_cache,
                   AffineTransform( Matrix2x2( scaling, 0, 0, scaling ),
                                    Vector2(-scaling*pt.x+20.5,
                                            -scaling*pt.y+20.5) ),
                   42, 42 );
      pt.descriptor.set_size( 180 );
      gen.compute_descriptor( patch, pt.begin(), pt.end() );
    }
  }

  // Matching
  {
    std::vector<ip::InterestPoint> ip_wac_v, ip_amc_v;
    std::copy( wac_ip.begin(), wac_ip.end(),
               std::back_inserter(ip_wac_v) );
    std::copy( trans_ip.begin(), trans_ip.end(),
               std::back_inserter(ip_amc_v) );
    ip::DefaultMatcher matcher(0.6);
    std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
    matcher(ip_wac_v, ip_amc_v, matched_ip1, matched_ip2, false,
            TerminalProgressCallback( "tools.ipalign", "Matching:"));
    remove_duplicates( matched_ip1, matched_ip2 );

    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
    std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);

    std::vector<size_t> indices;
    math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> ransac(math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 20);
    Matrix<double> align_matrix = ransac(ransac_ip2,ransac_ip1);
    if ( norm_2(subvector(select_col(align_matrix,2),0,2)) > 200 ||
         align_matrix(0,0) < 0 || align_matrix(1,1) < 0 ) {
      std::cout << "RANSAC FITTED TO OUTLIER\n";
      return 1;
    }

    std::cout << "Align Matrix: " << align_matrix << "\n";
    indices = ransac.inlier_indices(align_matrix,ransac_ip2,ransac_ip1);

    std::vector<ip::InterestPoint> final_ip1, final_ip2;
    for (size_t idx=0; idx < indices.size(); ++idx) {
      final_ip1.push_back(matched_ip1[indices[idx]]);
      final_ip2.push_back(matched_ip2[indices[idx]]);
    }

    if ( final_ip1.size() < 6 ) {
      std::cout << "FAILED TO FIND ENOUGH IPs\n";
      return 1;
    }
    std::cout << "Found " << final_ip1.size() << " points prior equalization.\n";

    // Equalizing matches
    //std::string oprefix = fs::path(cube_file ).stem();
    asp::cnettk::equalization( final_ip1, final_ip2, 20 );
    //ip::write_binary_match_file(oprefix+"debug.match", final_ip1, final_ip2);
    //write_image( oprefix+"debug1.tif", wac_cache );
    //write_image( oprefix+"debug2.tif", trans_cache );

    trans_ip.clear();
    wac_ip.clear();
    std::copy( final_ip1.begin(), final_ip1.end(),
               std::back_inserter(wac_ip) );
    std::copy( final_ip2.begin(), final_ip2.end(),
               std::back_inserter(trans_ip) );
  }

  // Build control network of measurements
  ba::ControlNetwork cnet("WAC LOLA GCPs v3",ba::ControlNetwork::ImageToGround);
  InterpolationView<EdgeExtensionView<ImageViewRef<double>, ConstantEdgeExtension>, BicubicInterpolation> current_lola_image =
    interpolate(ImageViewRef<double>(), BicubicInterpolation(), ConstantEdgeExtension());
  std::string current_lola_file;
  cartography::GeoReference current_lola_georef;
  for ( ip::InterestPointList::iterator trans_pt = trans_ip.begin(),
          wac_pt = wac_ip.begin(); trans_pt != trans_ip.end(); ++trans_pt, ++wac_pt ) {

    // Convert trans_pt to an actually AMC measurement
    Vector2f amc_pt = trans.reverse( Vector2f( trans_pt->x,
                                               trans_pt->y ) );
    // Convert WAC to actual lonlat location
    Vector2 wac_lonlat =
      wac_georef.pixel_to_lonlat( wac_trans.reverse( Vector2( wac_pt->x,
                                                              wac_pt->y ) ) );

    // Load corresponding LOLA data
    std::pair<cartography::GeoReference, std::string> result =
      lola_database.find_tile( wac_lonlat );
    if ( result.second != current_lola_file ) {
      std::cout << "Using LOLA tile: " << result.second << "\n";
      current_lola_file = result.second;
      current_lola_georef = result.first;
      current_lola_image =
        interpolate(ImageViewRef<double>(DiskImageView<double>(result.second)),
                    BicubicInterpolation(), ConstantEdgeExtension());
    }
    Vector2 lola_pt = current_lola_georef.lonlat_to_pixel(wac_lonlat);
    double radius = current_lola_image(lola_pt[0],lola_pt[1]) +
      current_lola_georef.datum().radius( wac_lonlat[0], wac_lonlat[1] );

    std::cout << amc_pt << " -> " << wac_lonlat << " " << radius << "\n";

    // Create the control point and measurement
    ba::ControlPoint cpoint(ba::ControlPoint::GroundControlPoint);
    cpoint.set_position(cartography::LonLatRadToXYZFunctor()(Vector3(wac_lonlat[0],
                                                                     wac_lonlat[1],
                                                                     radius)));
    double sigma = 160 * trans_pt->scale;
    cpoint.set_sigma(Vector3(sigma,sigma,sigma));
    ba::ControlMeasure cm( amc_pt[0], amc_pt[1], 1, 1,
                           ba::ControlMeasure::Automatic );
    cm.set_serial( serial_number );
    cm.set_image_id( 1 );
    cpoint.add_measure( cm );
    cnet.add_control_point( cpoint );
  }

  std::cout << "Stem: " << fs::path(cube_file).stem() << "\n";
  cnet.write_binary(fs::path(cube_file).stem()+"_lola_wac.cnet");

  return 0;
}
