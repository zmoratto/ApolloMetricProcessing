#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Cartography/SimplePointImageManipulation.h>
#include "LolaQuery.h"
#include <boost/foreach.hpp>

using namespace vw;

int main( int argc, char* argv[] ) {
  if ( argc != 2 )
    vw_throw( ArgumentErr() << "Expected: lola_relookup <cnet file>" );
  std::string cnet_file( argv[1] );

  // Open input files
  ba::ControlNetwork cnet( cnet_file, ba::FmtBinary );
  LOLAQuery lola_database;

  // Start cache
  std::string current_lola_file;
  InterpolationView<EdgeExtensionView<ImageViewRef<double>, ConstantEdgeExtension>, BicubicInterpolation> current_lola_image = interpolate(ImageViewRef<double>(), BicubicInterpolation(), ConstantEdgeExtension());

  // Modify Control Points
  size_t mod_count = 0;
  BOOST_FOREACH( ba::ControlPoint& cp, cnet ) {
    if ( cp.type() != ba::ControlPoint::GroundControlPoint )
      continue;
    mod_count++;
    Vector3 llr = cartography::xyz_to_lon_lat_radius(cp.position());

    std::pair<cartography::GeoReference,std::string> result =
      lola_database.find_tile( llr );
    if ( result.second != current_lola_file ) {
      current_lola_file = result.second;
      vw_out() << "Using LOLA tile: " << current_lola_file << "\n";
      current_lola_image =
        interpolate(ImageViewRef<double>(DiskImageView<double>(current_lola_file)),
                    BicubicInterpolation(), ConstantEdgeExtension());
    }

    Vector2 lola_px = result.first.lonlat_to_pixel( subvector(llr,0,2) );
    if ( !bounding_box(current_lola_image).contains(lola_px) )
      vw_out(WarningMessage) << "Extrapolating LOLA measurement";
    llr[2] = current_lola_image(lola_px[0],lola_px[1]) +
      result.first.datum().radius( llr[0], llr[1] );
    cp.set_position( cartography::lon_lat_radius_to_xyz( llr ) );
  }
  vw_out() << "\tModified " << mod_count << " GCPs.\n";

  vw_out() << "Writing: " << cnet_file << "\n";
  cnet.write_binary( cnet_file );

  return 0;
}
