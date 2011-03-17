#ifndef __CAMERA_SOLVE_H__
#define __CAMERA_SOLVE_H__

#include <vw/Math/LevenbergMarquardt.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>

namespace vw {

  class CameraGCPLMA : public math::LeastSquaresModelBase<CameraGCPLMA> {
    ba::ControlNetwork m_cnet;
    boost::shared_ptr<camera::IsisAdjustCameraModel> m_camera;

  public:
    // What is returned by evaluating functor. It's the reprojection error.
    typedef Vector<double> result_type;
    // Define the search space. This is the camera matrix flattened out.
    typedef Vector<double> domain_type;
    // Jacobian form
    typedef Matrix<double> jacobian_type;

    CameraGCPLMA( ba::ControlNetwork const& cnet,
                  boost::shared_ptr<camera::IsisAdjustCameraModel> cam ) :
      m_cnet(cnet), m_camera(cam) {
      VW_ASSERT( m_cnet.size() > 3,
                 IOErr() << "CameraGCPLMA: didn't recieve enough GCPs" );
    }


    // Evaluator. X is the new camera adjustments
    inline result_type operator()( domain_type const& x ) const {
      insert( m_camera, x );

      result_type output;
      output.set_size( m_cnet.size()*2 );
      size_t index = 0;
      BOOST_FOREACH( ba::ControlPoint const& cp, m_cnet ) {
        Vector2 error = m_camera->point_to_pixel( cp.position() ) -
          cp[0].position();
        subvector(output,index,2) = error;
        index++;
      }
      return output;
    }

    // Pulls out current variables
    inline domain_type extract( boost::shared_ptr<camera::IsisAdjustCameraModel> cam ) {
      domain_type output;
      boost::shared_ptr<asp::BaseEquation> position = cam->position_func();
      boost::shared_ptr<asp::BaseEquation> pose = cam->pose_func();
      output.set_size( position->size() + pose->size() );
      for ( size_t i = 0; i < position->size(); i++ )
        output[i] = (*position)[i];
      for ( size_t i = 0; i < pose->size(); i++ )
        output[i+position->size()] = (*pose)[i];
      return output;
    }

    // Insert domain into camera model
    inline void insert( boost::shared_ptr<camera::IsisAdjustCameraModel> cam,
                        domain_type const& x ) const {
      boost::shared_ptr<asp::BaseEquation> position = cam->position_func();
      boost::shared_ptr<asp::BaseEquation> pose = cam->pose_func();
      VW_ASSERT( x.size() == position->size() + pose->size(),
                 IOErr() << "CameraGCPLMA: domain_type doesn't match equations" );
      for ( size_t i = 0; i < position->size(); i++ )
        (*position)[i] = x[i];
      for ( size_t i = 0; i < pose->size(); i++ )
        (*pose)[i] = x[i+position->size()];
    }

  };

}

#endif//__CAMERA_SOLVE_H__
