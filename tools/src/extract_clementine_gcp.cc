#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
using namespace vw::ip;
using namespace vw::ba;

int main( int argc, char* argv[] ) {
  std::vector<std::string> input_file_names;
  po::options_description general_options("Options");
  general_options.add_options()
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

  std::string lola_names("/Users/zmoratto/Data/Moon/LOLA/LOLA_64px_p_deg_DEM.tif");

  std::vector<std::string> clementine_names;
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_30n045_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_30n135_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_30n225_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_30n315_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_30s045_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_30s135_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_30s225_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_30s315_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_75n090_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_75n270_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_75s090_256ppd.tif");
  clementine_names.push_back("/Users/zmoratto/Data/Moon/Clementine/BaseMapV2/clembase_75s270_256ppd.tif");

  // Load up those files georeference information
  std::vector<GeoReference> clementine_georef;
  std::vector<BBox2i> clementine_bbox;
  BOOST_FOREACH( std::string const& names, clementine_names ) {
    GeoReference georef;
    read_georeference( georef, names );
    clementine_georef.push_back(georef);
    DiskImageView<PixelGray<uint8> > dimage( names );
    clementine_bbox.push_back(bounding_box(dimage));
  }

  // Load up the camera models
  BOOST_FOREACH( std::string const& pinhole_name, input_file_names ) {
    PinholeModel pinhole( pinhole_name );

    Vector3 llr = XYZtoLonLatRadFunctor()( pinhole.camera_center() );
    std::string match_filename;

    // Finding an image that actually contains this camera's location
    for ( unsigned i = 0; i < clementine_names.size(); i++ ) {
      Vector2 clem_pix = clementine_georef[i].lonlat_to_pixel(subvector(llr,0,2));
      if ( clementine_bbox[i].contains(clem_pix) ) {
        // Create a new GeoReference
        GeoReference georef_dest;
        georef_dest.set_well_known_geogcs("D_MOON");
        georef_dest.set_orthographic( llr[1], llr[0] );
        Matrix<double> itx = vw::math::identity_matrix<3>();
        double zoom = 160;
        itx(0,0) = zoom;
        itx(1,1) = -zoom;
        //itx(0,2) = itx(1,2) = 00*zoom;
        itx(0,2) = -512*zoom;
        itx(1,2) = 512*zoom;
        georef_dest.set_transform(itx);
        GeoTransform geotx( clementine_georef[i], georef_dest );

        DiskImageView<PixelGray<uint8> > input(clementine_names[i]);
        ImageViewRef<PixelGray<uint8> > output =
          crop( transform( input, geotx, ZeroEdgeExtension(),
                           BicubicInterpolation() ), BBox2i(0,0,1024,1024) );


        write_georeferenced_image( fs::path(pinhole_name).replace_extension().string() +
                                   ".clem.tif", output, georef_dest,
                                   TerminalProgressCallback("test","Writing:"));

        break;
      }
    }

    { // Extracting interest points from the new clementine image
      std::string clementine_image =
        fs::path(pinhole_name).replace_extension(".clem.tif").string();
      std::string apollo_image =
        fs::path(pinhole_name).replace_extension(".tif").string();

      const float IDEAL_OBALOG_THRESHOLD = .07;
      InterestPointList ip_clem, ip_apollo;
      {
        OBALoGInterestOperator
          interest_operator(IDEAL_OBALOG_THRESHOLD/10);
        IntegralInterestPointDetector<OBALoGInterestOperator>
          detector( interest_operator );
        DiskImageView<PixelGray<uint8> > clem(clementine_image),
          apollo(apollo_image);
        ip_clem = detect_interest_points(clem, detector);
        ip_apollo = detect_interest_points(apollo, detector);
        SGradDescriptorGenerator descriptor;
        descriptor(clem, ip_clem);
        descriptor(apollo, ip_apollo);
      }

      try { // Matching
        std::vector<InterestPoint> ip_clem_v, ip_apollo_v;
        std::copy( ip_clem.begin(), ip_clem.end(),
                   std::back_inserter(ip_clem_v) );
        std::copy( ip_apollo.begin(), ip_apollo.end(),
                   std::back_inserter(ip_apollo_v) );
        DefaultMatcher matcher(0.7);
        std::vector<InterestPoint> matched_ip1, matched_ip2;
        matcher(ip_clem_v, ip_apollo_v, matched_ip1, matched_ip2, false,
                TerminalProgressCallback( "tools.ipalign", "Matching:"));

        std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
        std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
        Matrix<double> align_matrix;

        std::vector<int> indices;
        math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> ransac(math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 10);
        align_matrix = ransac(ransac_ip2,ransac_ip1);
        indices = ransac.inlier_indices(align_matrix,ransac_ip2,ransac_ip1);

        std::vector<InterestPoint> final_ip1, final_ip2;
        for (unsigned idx=0; idx < indices.size(); ++idx) {
          final_ip1.push_back(matched_ip1[indices[idx]]);
          final_ip2.push_back(matched_ip2[indices[idx]]);
        }
        match_filename =
          fs::path(clementine_image).replace_extension().string() + "__" +
          fs::path(apollo_image).stem() + ".match";
        write_binary_match_file(match_filename, final_ip1, final_ip2);
        if ( final_ip1.size() < 8 ) {
          std::cout << "FAILED TO FIND ENOUGH IPs\n";
          continue;
        }
      }  catch ( ... ) {
        std::cout << "RANSAC FAILED\n";
        continue;
      }
    }

    { // Looking up pixel locations into GCP positions
      std::vector<InterestPoint> ip1, ip2; // First one is on the clementine
      read_binary_match_file(match_filename, ip1, ip2);
      std::string clementine_image =
        fs::path(pinhole_name).replace_extension(".clem.tif").string();
      GeoReference clem_georef, lola_georef;
      read_georeference( clem_georef, clementine_image );
      read_georeference( lola_georef, lola_names );

      DiskImageView<double> lola(lola_names);
      ImageViewRef<double> lola_intrp =
        interpolate(lola,BicubicInterpolation(),ZeroEdgeExtension());

      ControlNetwork cnet(pinhole_name,ControlNetwork::ImageToGround);
      for ( unsigned i = 0; i < ip1.size() && i < 10; i++ ) {
        Vector2 lonlat = clem_georef.pixel_to_lonlat(Vector2(ip1[i].x,ip1[i].y));
        Vector2 lola_px = lola_georef.lonlat_to_pixel(lonlat);
        double radius = lola_intrp(lola_px[0],lola_px[1]) +
          lola_georef.datum().radius( lonlat[0], lonlat[1] );

        ControlPoint cpoint(ControlPoint::GroundControlPoint);
        cpoint.set_position(LonLatRadToXYZFunctor()(Vector3(lonlat[0],lonlat[1],radius)));
        cpoint.set_sigma(Vector3(300,300,300));
        ControlMeasure cm(ip2[i].x*5725./1024.,
                          ip2[i].y*5725./1024.,
                          5,5,0);
        cm.set_serial(fs::path(pinhole_name).replace_extension().string());
        cpoint.add_measure(cm);
        cnet.add_control_point(cpoint);
      } // end for loop
      std::cout << "CNET size: " << cnet.size() << "\n";
      std::cout << "Control Measure: " << cnet[0] << "\n";
      std::string cnet_file =
        fs::path(pinhole_name).replace_extension("cnet").string();
      std::cout << "Writing: " << cnet_file << "\n";
      cnet.write_binary(cnet_file);
    }
  }
}
