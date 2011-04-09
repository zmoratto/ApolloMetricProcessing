/* This is version 2, which attempts to extract data from a much
   higher resolution version of LOLA.
 */
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Core/Debugging.h>

#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/ControlNetTK/equalization.h>

#include <boost/foreach.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

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
class ComputeNormalsFunc : public ReturnFixedType<PixelMask<Vector3> >
{
  double m_u_scale, m_v_scale;

public:
  ComputeNormalsFunc(double u_scale, double v_scale) :
    m_u_scale(u_scale), m_v_scale(v_scale) {}

  BBox2i work_area() const { return BBox2i(Vector2i(0, 0), Vector2i(1, 1)); }

  template <class PixelAccessorT>
  PixelMask<Vector3> operator() (PixelAccessorT const& accessor_loc) const {
    PixelAccessorT acc = accessor_loc;

    // Pick out the three altitude values.
    if (is_transparent(*acc))
      return PixelMask<Vector3>();
    double alt1 = *acc;

    acc.advance(1,0);
    if (is_transparent(*acc))
      return PixelMask<Vector3>();
    double alt2 = *acc;

    acc.advance(-1,1);
    if (is_transparent(*acc))
      return PixelMask<Vector3>();
    double alt3 = *acc;

    // Form two orthogonal vectors in the plane containing the three
    // altitude points
    Vector3 n1(m_u_scale, 0, alt2-alt1);
    Vector3 n2(0, m_v_scale, alt3-alt1);

    // Return the vector normal to the local plane.
    return normalize(cross_prod(n1,n2));
  }
};

template <class ViewT>
UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, ComputeNormalsFunc> compute_normals(ImageViewBase<ViewT> const& image,
                                                                                                              double u_scale, double v_scale) {
  return UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, ComputeNormalsFunc>(edge_extend(image.impl(), ConstantEdgeExtension()),
                                                                                                       ComputeNormalsFunc (u_scale, v_scale));
}

class DotProdFunc : public ReturnFixedType<PixelMask<PixelGray<double> > > {
  Vector3 m_vec;
public:
  DotProdFunc(Vector3 const& vec) : m_vec(vec) {}
  PixelMask<PixelGray<double> > operator() (PixelMask<Vector3> const& pix) const {
    if (is_transparent(pix))
      return PixelMask<PixelGray<double> >();
    else {
//       std::cout << "Vec1 : " << pix.child() << "   " << norm_2(pix.child()) << "\n";
//       std::cout << "Vec2 : " << m_vec << "   " << norm_2(m_vec) << "\n";
//       std::cout << "OVerall: " << dot_prod(pix.child(),m_vec)/(norm_2(pix.child()) * norm_2(m_vec)) << "\n\n";
      return dot_prod(pix.child(),m_vec)/(norm_2(pix.child()) * norm_2(m_vec));
    }
  }
};

template <class ViewT>
UnaryPerPixelView<ViewT, DotProdFunc> dot_prod(ImageViewBase<ViewT> const& view, Vector3 const& vec) {
  return UnaryPerPixelView<ViewT, DotProdFunc>(view.impl(), DotProdFunc(vec));
}

class LOLAQuery {
  // These should be hard coded
  std::vector<std::string> m_filenames;
  std::vector<BBox2> m_bboxes;
public:
  LOLAQuery() {
    std::string base_path("/Users/zmoratto/Data/Moon/LOLA/LOLA_DEM_1024/");
    // Right
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,0),Vector2(30,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,0),Vector2(60,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,0),Vector2(90,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,0),Vector2(120,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,0),Vector2(150,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,0),Vector2(180,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,0),Vector2(210,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,0),Vector2(240,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,0),Vector2(270,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,0),Vector2(300,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,0),Vector2(330,15)));
    m_filenames.push_back(base_path+"LDEM_1024_00N_15N_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,0),Vector2(360,15)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,15),Vector2(30,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,15),Vector2(60,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,15),Vector2(90,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,15),Vector2(120,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,15),Vector2(150,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,15),Vector2(180,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,15),Vector2(210,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,15),Vector2(240,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,15),Vector2(270,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,15),Vector2(300,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,15),Vector2(330,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15N_30N_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,15),Vector2(360,30)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,-15),Vector2(30,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,-15),Vector2(60,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,-15),Vector2(90,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,-15),Vector2(120,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,-15),Vector2(150,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,-15),Vector2(180,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,-15),Vector2(210,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,-15),Vector2(240,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,-15),Vector2(270,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,-15),Vector2(300,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,-15),Vector2(330,0)));
    m_filenames.push_back(base_path+"LDEM_1024_15S_00S_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,-15),Vector2(360,0)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,30),Vector2(30,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,30),Vector2(60,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,30),Vector2(90,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,30),Vector2(120,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,30),Vector2(150,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,30),Vector2(180,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,30),Vector2(210,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,30),Vector2(240,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,30),Vector2(270,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,30),Vector2(300,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,30),Vector2(330,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30N_45N_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,30),Vector2(360,45)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,-30),Vector2(30,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,-30),Vector2(60,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,-30),Vector2(90,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,-30),Vector2(120,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,-30),Vector2(150,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,-30),Vector2(180,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,-30),Vector2(210,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,-30),Vector2(240,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,-30),Vector2(270,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,-30),Vector2(300,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,-30),Vector2(330,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_30S_15S_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,-30),Vector2(360,-15)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,45),Vector2(30,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,45),Vector2(60,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,45),Vector2(90,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,45),Vector2(120,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,45),Vector2(150,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,45),Vector2(180,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,45),Vector2(210,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,45),Vector2(240,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,45),Vector2(270,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,45),Vector2(300,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,45),Vector2(330,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45N_60N_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,45),Vector2(360,60)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,-45),Vector2(30,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,-45),Vector2(60,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,-45),Vector2(90,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,-45),Vector2(120,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,-45),Vector2(150,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,-45),Vector2(180,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,-45),Vector2(210,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,-45),Vector2(240,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,-45),Vector2(270,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,-45),Vector2(300,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,-45),Vector2(330,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_45S_30S_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,-45),Vector2(360,-30)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,60),Vector2(30,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,60),Vector2(60,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,60),Vector2(90,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,60),Vector2(120,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,60),Vector2(150,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,60),Vector2(180,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,60),Vector2(210,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,60),Vector2(240,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,60),Vector2(270,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,60),Vector2(300,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,60),Vector2(330,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60N_75N_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,60),Vector2(360,75)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,-60),Vector2(30,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,-60),Vector2(60,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,-60),Vector2(90,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,-60),Vector2(120,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,-60),Vector2(150,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,-60),Vector2(180,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,-60),Vector2(210,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,-60),Vector2(240,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,-60),Vector2(270,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,-60),Vector2(300,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,-60),Vector2(330,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_60S_45S_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,-60),Vector2(360,-45)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,75),Vector2(30,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,75),Vector2(60,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,75),Vector2(90,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,75),Vector2(120,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,75),Vector2(150,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,75),Vector2(180,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,75),Vector2(210,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,75),Vector2(240,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,75),Vector2(270,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,75),Vector2(300,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,75),Vector2(330,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75N_90N_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,75),Vector2(360,90)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,-75),Vector2(30,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,-75),Vector2(60,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,-75),Vector2(90,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,-75),Vector2(120,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,-75),Vector2(150,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,-75),Vector2(180,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,-75),Vector2(210,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,-75),Vector2(240,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,-75),Vector2(270,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,-75),Vector2(300,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,-75),Vector2(330,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_75S_60S_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,-75),Vector2(360,-60)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_000_030.tif");
    m_bboxes.push_back(BBox2(Vector2(0,-90),Vector2(30,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_030_060.tif");
    m_bboxes.push_back(BBox2(Vector2(30,-90),Vector2(60,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_060_090.tif");
    m_bboxes.push_back(BBox2(Vector2(60,-90),Vector2(90,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_090_120.tif");
    m_bboxes.push_back(BBox2(Vector2(90,-90),Vector2(120,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_120_150.tif");
    m_bboxes.push_back(BBox2(Vector2(120,-90),Vector2(150,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_150_180.tif");
    m_bboxes.push_back(BBox2(Vector2(150,-90),Vector2(180,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_180_210.tif");
    m_bboxes.push_back(BBox2(Vector2(180,-90),Vector2(210,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_210_240.tif");
    m_bboxes.push_back(BBox2(Vector2(210,-90),Vector2(240,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_240_270.tif");
    m_bboxes.push_back(BBox2(Vector2(240,-90),Vector2(270,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_270_300.tif");
    m_bboxes.push_back(BBox2(Vector2(270,-90),Vector2(300,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_300_330.tif");
    m_bboxes.push_back(BBox2(Vector2(300,-90),Vector2(330,-75)));
    m_filenames.push_back(base_path+"LDEM_1024_90S_75S_330_360.tif");
    m_bboxes.push_back(BBox2(Vector2(330,-90),Vector2(360,-75)));

    VW_ASSERT( m_filenames.size() == 144, LogicErr() << "Programmer should provide 144 inputs for entire coverage of Moon" );
    VW_ASSERT( m_bboxes.size() == 144, LogicErr() << "Programmer should provide 144 inputs for entire coverage of Moon" );
  }

  void print() {
    for ( size_t i = 0; i < m_bboxes.size(); i++ ) {
      std::cout << "File: " << m_filenames[i] << "\n";
      std::cout << "BBox: " << m_bboxes[i] << "\n";
    }
  }

  std::pair<cartography::GeoReference, std::string>
  find_tile( Vector3 llr ) {
    if ( llr[0] < 0 )
      llr[0] += 360;
    std::cout << "Query: " << llr << "\n";
    for ( size_t i = 0; i < m_bboxes.size(); i++ ) {
      if ( m_bboxes[i].contains( subvector(llr,0,2) ) ) {
        std::pair<cartography::GeoReference, std::string> result;
        cartography::read_georeference( result.first,
                                        m_filenames[i] );
        result.second = m_filenames[i];
        return result;
      }
    }
    vw_throw( ArgumentErr() << "Unable to find match?" );
  }
};

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
  Matrix<double> tx = math::identity_matrix<3>();
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
  {
    Timer cow("Test Apollo Lighting: ");
    write_image( "test.tif", gaussian_filter(channel_cast_rescale<uint8>(clamp(dot_prod(compute_normals(local_dem,
                                                                                                        pixel_meters,pixel_meters),
                                                                                        sun_position))),1.5),
                 TerminalProgressCallback("tools","Saving:") );
  }

  // Guess the WAC mosaic lighting
  Vector3 wac_light = math::rotation_z_axis(rotate)*Vector3(-3,0,1);
  wac_light = normalize(wac_light);
  std::cout << "Wac Normal: " << wac_light << "\n";
  std::cout << "Sun Normal: " << sun_position << "\n";
  {
    Timer cow("Test WAC Lighting: ");
    write_image( "wac_test.tif", gaussian_filter(channel_cast_rescale<uint8>(clamp(dot_prod(compute_normals(local_dem,
                                                                                                            pixel_meters, pixel_meters),
                                                                                            wac_light))),1.5),
                 TerminalProgressCallback("tools","Saving:") );
  }

  return 0;
}
