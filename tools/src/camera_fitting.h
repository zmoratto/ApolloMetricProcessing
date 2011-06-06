#ifndef __CAMERA_FITTING_H__
#define __CAMERA_FITTING_H__

#include <vw/Math/EulerAngles.h>
#include <vw/Camera.h>

namespace vw {

  struct CAHVOptimizeFunctor : public math::LeastSquaresModelBase<CAHVOptimizeFunctor> {
    typedef Vector<double,18> result_type;
    typedef Vector<double,9>  domain_type;
    typedef Matrix<double>  jacobian_type;

    camera::CameraModel* m_cam;
    const std::vector<Vector2> m_measure;
    CAHVOptimizeFunctor(camera::CameraModel* cam,
                        std::vector<Vector2> const& measure ) : m_cam(cam), m_measure(measure) {}

    inline result_type operator()( domain_type const& x ) const {
      camera::CAHVModel cahv(m_cam->camera_center(Vector2()),
                             subvector(x,0,3), subvector(x,3,3),
                             subvector(x,6,3) );
      cahv.A = normalize(cahv.A);
      result_type output;
      for ( size_t i = 0; i < 9; i++ )
        subvector(output,2*i,2) = m_measure[i] -
          cahv.point_to_pixel(cahv.C + 1000*m_cam->pixel_to_vector(m_measure[i]) );
      return output;
    }
  };

  struct PinholeOptimizeFunctor : public math::LeastSquaresModelBase<PinholeOptimizeFunctor> {
    typedef Vector<double, 18> result_type;
    typedef Vector<double, 3>  domain_type;
    typedef Matrix<double>     jacobian_type;

    camera::CameraModel* m_cam;
    const std::vector<Vector2> m_measure;
    PinholeOptimizeFunctor( camera::CameraModel* cam,
                            std::vector<Vector2> const& measure ) : m_cam(cam), m_measure(measure) {}

    inline result_type operator()( domain_type const& x ) const {
      camera::PinholeModel ccam =
        to_pinhole( x, m_cam->camera_center(Vector2()) );
      result_type output;
      for ( size_t i = 0; i < 9; i++ )
        subvector(output,2*i,2) = m_measure[i] -
          ccam.point_to_pixel(m_cam->camera_center(Vector2()) +
                              1000*m_cam->pixel_to_vector(m_measure[i]));
      return output;
    }

    static domain_type to_vec( camera::PinholeModel const& model ) {
      domain_type vec;
      vec[0] = model.focal_length()[0];
      subvector( vec, 1, 2 ) = model.point_offset();
      return vec;
    }

    camera::PinholeModel to_pinhole( domain_type const& vec,
                                     Vector3 const& center ) const {
      return camera::PinholeModel( center, m_cam->camera_pose(Vector2()).rotation_matrix(),
                                   vec[0], vec[0], vec[1], vec[2],
                                   Vector3(1,0,0), Vector3(0,1,0),
                                   Vector3(0,0,1),
                                   camera::NullLensDistortion() );
    }
  };

  camera::PinholeModel linearize_pinhole( camera::CameraModel* cam,
                                          Vector2i const& size ) {
    Vector2 center_px  = (size - Vector2(1,1))/2.0;
    Vector3 center_vec = cam->pixel_to_vector( center_px );
    Vector3 off_x_vec  = cam->pixel_to_vector( center_px + Vector2i(10,0) );
    Vector3 off_y_vec  = cam->pixel_to_vector( center_px + Vector2i(0,10) );

    double fu = 10.0 / tan( acos( dot_prod(off_x_vec, center_vec ) ) );
    double fv = 10.0 / tan( acos( dot_prod(off_y_vec, center_vec ) ) );

    camera::PinholeModel initial( cam->camera_center(center_px), cam->camera_pose(center_px).rotation_matrix(),
                          fu, fv, -size[0]/2, -size[1]/2 );
    initial.set_coordinate_frame( Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1) );

    std::vector<Vector2> input(9);
    input[0] = Vector2();
    input[1] = Vector2(size[0]/2,0);
    input[2] = Vector2(size[0]-1,0);
    input[3] = Vector2(size[0]-1,size[1]/2);
    input[4] = Vector2(size[0]-1,size[1]-1);
    input[5] = Vector2(size[0]/2,size[1]-1);
    input[6] = Vector2(0        ,size[1]-1);
    input[7] = Vector2(0        ,size[1]/2);
    input[8] = center_px;

    int status;
    Vector<double> seed = PinholeOptimizeFunctor::to_vec( initial );
    PinholeOptimizeFunctor model( cam, input );
    Vector<double> sol =
      math::levenberg_marquardt( model, seed, Vector<double,18>(), status );
    std::cout << "Status  : " << status << "\n";
    std::cout << "Error   : " << norm_2( PinholeOptimizeFunctor(cam,input)(sol) ) << "\n";
    return model.to_pinhole( sol, cam->camera_center(center_px) );
  }

  camera::CAHVModel linearize_camera( camera::CameraModel* cam, Vector2i const& size ) {
    camera::CAHVModel output;
    Vector2 center_px = (size - Vector2(1,1))/2.0;
    output.C = cam->camera_center(Vector2());
    output.A = cam->pixel_to_vector( center_px );
    output.H =
      normalize(cross_prod(cam->pixel_to_vector(center_px+Vector2(0,1)),
                           cam->pixel_to_vector(center_px)));
    output.V =
      normalize(cross_prod(cam->pixel_to_vector(center_px),
                           cam->pixel_to_vector(center_px+Vector2(1,0))));

    double h_angle =
      M_PI/2 - acos(dot_prod(cam->pixel_to_vector(Vector2(size[0]-1,center_px[1])),
                             output.H));
    double h_focal = size[0]*0.5/tan(h_angle);

    double v_angle =
      M_PI/2 - acos(dot_prod(cam->pixel_to_vector(Vector2(center_px[0],size[1]-1)),
                             output.V));
    double v_focal = size[1]*0.5/tan(v_angle);


    output.H = Vector3(-(size[0]-1)/2,0,h_focal);
    output.V = Vector3(-(size[1]-1)/2,0,v_focal);

    std::vector<Vector2> input(9);
    input[0] = Vector2();
    input[1] = Vector2(size[0]/2,0);
    input[2] = Vector2(size[0]-1,0);
    input[3] = Vector2(size[0]-1,size[1]/2);
    input[4] = Vector2(size[0]-1,size[1]-1);
    input[5] = Vector2(size[0]/2,size[1]-1);
    input[6] = Vector2(0        ,size[1]-1);
    input[7] = Vector2(0        ,size[1]/2);
    input[8] = center_px;

    Vector<double,9> seed;
    subvector(seed,0,3) = output.A;
    subvector(seed,3,3) = output.H;
    subvector(seed,6,3) = output.V;
    std::cout << "Seed: " << seed << "\n";

    int status;
    Vector<double> camera =
      math::levenberg_marquardt( CAHVOptimizeFunctor( cam, input ),
                                 seed, Vector<double,18>(), status );
    output.A = normalize(subvector(camera,0,3));
    output.H = subvector(camera,3,3);
    output.V = subvector(camera,6,3);
    std::cout << "Status  : " << status << "\n";
    std::cout << "Error   : " << norm_2( CAHVOptimizeFunctor(cam,input)(camera) ) << "\n";

    return output;
  }

}

#endif//__CAMERA_FITTING_H__
