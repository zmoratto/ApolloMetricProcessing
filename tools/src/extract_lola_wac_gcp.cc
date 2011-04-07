#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/Geometry.h>

#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/ControlNetTK/equalization.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;
#include <boost/foreach.hpp>

#include "camera_solve.h"

using namespace vw;
using namespace vw::camera;

inline Vector2 center_on_180( Vector2 vec ) {
  if ( vec[0] < 0 )
    vec[0] += 360.0;
  if ( vec[0] > 360 )
    vec[0] -= 360.0;
  return vec;
}

int main( int argc, char* argv[] ) {
  std::vector<std::string> input_file_names;
  po::options_description general_options("Options");
  general_options.add_options()
    ("reuse-wac", "reuse the wac, trust")
    ("generate-wac-only", "Generate wac crops only")
    ("match-only", "Only perform matching")
    ("no-match", "No matching please .. only loading")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_file_names));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <image-files> ...\n\n";
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str() << std::endl;
    return 1;
  } else if ( input_file_names.size() == 0 ) {
    vw_out() << "ERROR! Require an input file.\n";
    vw_out() << "\n" << usage.str() << "\n";
    return 1;
  }

  // CONSTANTS for maps. Sorry guys this is specific for my system.
  // The WAC mosaic is 20GB and is currently private.
  std::string lola_file("/Users/zmoratto/Data/Moon/LOLA/LOLA_DEM_64/LOLA_64px_p_deg_DEM.tif");
  std::string wac_file("/Users/zmoratto/Data/Moon/LROWAC/global_100m_JanFeb_and_JulyAug.180.cub");
  cartography::GeoReference lola_georef, wac_georef;
  cartography::read_georeference( lola_georef, lola_file );
  cartography::read_georeference( wac_georef,  wac_file );

  // Create Control Network
  ba::ControlNetwork cnet("WAC LOLA GCPs",ba::ControlNetwork::ImageToGround);

  // Generating Data
  size_t image_id = 0;
  BOOST_FOREACH( std::string const& camera_file, input_file_names ) {
    // Create Camera Model
    boost::shared_ptr<CameraModel> model;
    std::string adjust_file =
      fs::path( camera_file ).replace_extension("isis_adjust").string();
    if ( fs::exists( adjust_file ) ) {
      vw_out() << "Loading \"" << adjust_file << "\"\n";
      std::ifstream input( adjust_file.c_str() );
      boost::shared_ptr<asp::BaseEquation> posF = asp::read_equation(input);
      boost::shared_ptr<asp::BaseEquation> poseF = asp::read_equation(input);
      input.close();
      model = boost::shared_ptr<CameraModel>(new IsisAdjustCameraModel( camera_file, posF, poseF ) );
    } else {
      vw_out() << "Loading \"" << camera_file << "\"\n";
      model = boost::shared_ptr<CameraModel>(new IsisCameraModel( camera_file ) );
    }

    // Extract serials
    std::string serial;
    Vector2i size;
    if ( fs::exists( adjust_file ) ) {
      boost::shared_ptr<IsisAdjustCameraModel> cam =
        boost::shared_dynamic_cast< IsisAdjustCameraModel >(model);
      serial = cam->serial_number();
      size = Vector2i(cam->samples(),cam->lines());
    } else {
      boost::shared_ptr<IsisCameraModel> cam =
        boost::shared_dynamic_cast< IsisCameraModel >(model);
      serial = cam->serial_number();
      size = Vector2i(cam->samples(),cam->lines());
    }

    // Working out scale and rotation
    float degree_scale;
    BBox2 degree_bbox =
      cartography::camera_bbox( lola_georef, model, size[0], size[1], degree_scale );

    // Correct bbox if it crosses the 180
    // camera_bbox has basically failed me here.
    if ( ( degree_bbox.max()[0] - degree_bbox.min()[0] ) > 180 ) {
      bool work;
      using namespace cartography;
      degree_bbox = BBox2();
      degree_bbox.grow( center_on_180( geospatial_intersect( Vector2(), lola_georef, model, 1, work ) ) );
      degree_bbox.grow( center_on_180( geospatial_intersect( Vector2(0,size[1]), lola_georef, model, 1, work ) ) );
      degree_bbox.grow( center_on_180( geospatial_intersect( Vector2(size[0],0), lola_georef, model, 1, work ) ) );
      degree_bbox.grow( center_on_180( geospatial_intersect( size, lola_georef, model, 1, work ) ) );
    }

    degree_bbox.expand(2);
    bool working;
    Vector2 l_direction =
      cartography::geospatial_intersect( Vector2(), lola_georef, model,
                                         1, working ) -
      cartography::geospatial_intersect( Vector2(0,size[1]), lola_georef, model,
                                         1, working );
    if (l_direction[0] < -200)
      l_direction[0] += 360;
    if (l_direction[0] > 200 )
      l_direction[0] -= 360;
    std::cout << "L direction: " << l_direction << "\n";
    Vector2 r_direction =
      cartography::geospatial_intersect( Vector2(size[0],0), lola_georef, model,
                                         1, working ) -
      cartography::geospatial_intersect( size, lola_georef, model,
                                         1, working );
    if (r_direction[0] < -200)
      r_direction[0] += 360;
    if (r_direction[0] > 200)
      r_direction[0] -= 360;
    std::cout << "R direction: " << r_direction << "\n";
    double rotate = M_PI/2 - (atan2(l_direction[1],l_direction[0]) +
                     atan2(r_direction[1],r_direction[0]) )/2;
    degree_scale = (norm_2(l_direction)+norm_2(r_direction)) / (size[1]*2);

    // Debug information
    vw_out() << "BBox: " << degree_bbox << "\n";
    vw_out() << "Scle: " << degree_scale << "\n"; // Degrees / pixel I assume
    vw_out() << "Rot : " << rotate*180/M_PI << "\n\n";

    // Creating output georef
    cartography::GeoReference georef_out;
    georef_out.set_datum( cartography::Datum("Sphere","Sphere","Zero",
                                             180.0/M_PI, 180.0/M_PI, 0) );
    georef_out.set_orthographic( degree_bbox.min()[1] + degree_bbox.height()/2,
                                 degree_bbox.min()[0] + degree_bbox.width()/2 );
    // Transform defines pixel to point
    Matrix<double> tx = math::identity_matrix<3>();
    tx(0,2) = -size[0]*degree_scale/2;
    tx(1,2) = size[1]*degree_scale/2;
    tx(0,0) = degree_scale;
    tx(1,1) = -degree_scale;
    georef_out.set_transform( tx );
    std::cout << "Georef: " << georef_out << "\n";
    TransformRef wactx( compose( RotateTransform( -rotate,
                                                  (Vector2(size)-Vector2(1,1))/2 ),
                                 cartography::GeoTransform( wac_georef, georef_out ) ) );

    // Rastering an image to perform transform
    if ( !vm.count("reuse-wac") ) {
      DiskImageView<PixelGray<uint8> > input( wac_file );
      ImageViewRef<PixelGray<uint8> > output1 =
        crop( transform( input, wactx ),
              BBox2i(0,0,size[0],size[1]) );
      write_image( fs::path(camera_file).replace_extension(".wac.tif").string(),
                   normalize(output1) );
    }
    if ( vm.count("generate-wac-only") )
      continue; // Finish for this file

    // Process image for interest points
    std::string wac_file =
      fs::path(camera_file).replace_extension(".wac.tif").string();
    std::string amc_file =
      fs::path(camera_file).replace_extension(".tif").string();

    std::string match_file =
      fs::path( wac_file ).stem() + "__" +
      fs::path( amc_file ).stem() + ".match";
    if ( (!fs::exists( match_file ) ||
          vm.count("match-only")) && !vm.count("no-match") ) {
      const float IDEAL_OBALOG_THRESHOLD = .07;
      ip::InterestPointList ip_wac, ip_amc;
      {
        ip::OBALoGInterestOperator
          interest_operator(IDEAL_OBALOG_THRESHOLD/3);
        ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator>
          detector( interest_operator );
        DiskImageView<PixelGray<uint8> > wac(wac_file),
          amc(amc_file);
        vw_out() << "Detecting Interest Points .... ";
        vw_out() << std::flush;
        ip_wac = detect_interest_points(wac, detector);
        ip_amc = detect_interest_points(amc, detector);
        ip::SGradDescriptorGenerator descriptor;
        descriptor(wac, ip_wac);
        descriptor(amc, ip_amc);
        vw_out() << "done\n";
      }

      try {
        std::vector<ip::InterestPoint> ip_wac_v, ip_amc_v;
        std::copy( ip_wac.begin(), ip_wac.end(),
                   std::back_inserter(ip_wac_v) );
        std::copy( ip_amc.begin(), ip_amc.end(),
                   std::back_inserter(ip_amc_v) );
        ip::DefaultMatcher matcher(0.6);
        std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
        matcher(ip_wac_v, ip_amc_v, matched_ip1, matched_ip2, false,
                TerminalProgressCallback( "tools.ipalign", "Matching:"));

        std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
        std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
        Matrix<double> align_matrix;

        std::vector<int> indices;
        math::RandomSampleConsensus<math::SimilarityFittingFunctor, math::InterestPointErrorMetric> ransac(math::SimilarityFittingFunctor(), math::InterestPointErrorMetric(), 10);
        align_matrix = ransac(ransac_ip2,ransac_ip1);
        if ( norm_2(subvector(select_col(align_matrix,2),0,2)) > 200 ||
             align_matrix(0,0) < 0 || align_matrix(1,1) < 0 ) {
          std::cout << "RANSAC FITTED TO OUTLIER\n";
          continue;
        }
        std::cout << "Align Matrix: " << align_matrix << "\n";
        indices = ransac.inlier_indices(align_matrix,ransac_ip2,ransac_ip1);

        std::vector<ip::InterestPoint> final_ip1, final_ip2;
        for (unsigned idx=0; idx < indices.size(); ++idx) {
          final_ip1.push_back(matched_ip1[indices[idx]]);
          final_ip2.push_back(matched_ip2[indices[idx]]);
        }

        if ( final_ip1.size() < 6 ) {
          std::cout << "FAILED TO FIND ENOUGH IPs\n";
          continue;
        }
        std::cout << "Found " << final_ip1.size() << " points prior equalization.\n";

        // Equalizing matches
        asp::cnettk::equalization( final_ip1, final_ip2, 10 );

        ip::write_binary_match_file(match_file, final_ip1, final_ip2);
      } catch ( ... ) {
        std::cout << "RANSAC FAILED\n";
        continue;
      }
    } // end of ip detection

    if ( fs::exists(match_file) ) { // Looking up LLA location of GCP files
      std::vector<ip::InterestPoint> wac_ip, amc_ip;
      ip::read_binary_match_file(match_file, wac_ip, amc_ip);

      DiskImageView<double> lola( lola_file );
      InterpolationView<EdgeExtensionView<DiskImageView<double>, PeriodicEdgeExtension>, BicubicInterpolation> lola_intrp( edge_extend(lola, PeriodicEdgeExtension() ), BicubicInterpolation() );

      for ( size_t i = 0; i < wac_ip.size(); i++ ) {
        Vector2 lonlat =
          wac_georef.pixel_to_lonlat( wactx.reverse(Vector2(wac_ip[i].x,wac_ip[i].y)) );

        Vector2 lola_px =  lola_georef.lonlat_to_pixel(lonlat);
        double radius = lola_intrp( lola_px[0], lola_px[1]) +
          lola_georef.datum().radius( lonlat[0], lonlat[1] );

        std::cout << i << "\t" << lonlat << " " << radius << "\n";

        // Actually appending to the Control Network.
        ba::ControlPoint cpoint(ba::ControlPoint::GroundControlPoint);
        cpoint.set_position(cartography::LonLatRadToXYZFunctor()(Vector3(lonlat[0],lonlat[1],radius)));
        cpoint.set_sigma(Vector3(300,300,300));
        ba::ControlMeasure cm( amc_ip[i].x, amc_ip[i].y, 1, 1,
                               ba::ControlMeasure::Automatic );
        cm.set_serial( serial );
        cm.set_image_id( image_id );
        cpoint.add_measure( cm );
        //std::cout << "Added:\n" << cm << "\n" << cpoint << "\n";
        cnet.add_control_point( cpoint );
      }
    }
    image_id++;

    // The sanity check code doesn't work with control networks
    // attached to other cameras.
    if ( input_file_names.size() > 1 )
      continue;

    // Sanity check. Let's solve for the camera position from these
    // points. Hopefully it doesn't move to far?
    boost::shared_ptr<IsisAdjustCameraModel> adjust_cam =
      boost::shared_dynamic_cast<IsisAdjustCameraModel>(model);
    if ( adjust_cam != 0 ) {
      CameraGCPLMA lma_model( cnet, adjust_cam );
      Vector<double> seed = lma_model.extract( adjust_cam );
      int status = 0;
      Vector<double> objective;
      objective.set_size( cnet.size() * 2 );
      Vector<double> result = levenberg_marquardt( lma_model, seed,
                                                   objective, status );
      std::cout << "Status: " << status << std::endl;
      std::cout << "Change: " << result - seed << std::endl;
    }
  }

  // Actually write the Control Network
  if ( !vm.count("match-only") && !vm.count("generate-wac-only") ) {
    std::cout << "Writing final control network: lola_wac_gcp.cnet\n";
    cnet.write_binary("lola_wac_gcp");
  }
}
