#ifndef __NVM_IO_H__
#define __NVM_IO_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <boost/filesystem/convenience.hpp>
#include <boost/foreach.hpp>

namespace vw {

  // Writing functions
  void write_nvm_camera(std::ostream& stream,
                        camera::PinholeModel const* cam) {
    stream << "unknown " << cam->focal_length()[0] << " ";
    Matrix3x3 rot = transpose(cam->camera_pose().rotation_matrix());
    stream << rot(0,0) << " " << rot(0,1) << " " << rot(0,2) << " "
           << rot(1,0) << " " << rot(1,1) << " " << rot(1,2) << " "
           << rot(2,0) << " " << rot(2,1) << " " << rot(2,2) << " ";
    Vector3 trans = -(rot*cam->camera_center());
    stream << trans[0] << " " << trans[1] << " " << trans[2] << " 0 0\n";
  }
  template <class RotT, class TransT>
  void write_nvm_r9t(std::ostream& stream,
                     float const& focal,
                     RotT const& rotation,
                     TransT const& translation) {
    stream << "unknown " << focal << " ";
    stream << rotation(0,0) << " " << rotation(0,1) << " "
           << rotation(0,2) << " " << rotation(1,0) << " "
           << rotation(1,1) << " " << rotation(1,2) << " "
           << rotation(2,0) << " " << rotation(2,1) << " "
           << rotation(2,2) << " ";
    stream << translation[0] << " " << translation[1] << " "
           << translation[2] << " 0 0\n";
  }
  void write_nvm_measurement(std::ostream& stream,
                             ba::ControlPoint const& cp) {
    stream << cp.position()[0] << " " << cp.position()[1] << " "
           << cp.position()[2] << " 0 0 0 " << cp.size();
    BOOST_FOREACH( ba::ControlMeasure const& cm, cp ) {
      stream << " " << cm.image_id() << " 0 "
             << cm.position()[0] << " " << cm.position()[1];
    }
    stream << "\n";
  }
  void write_nvm_controlnetwork( std::ostream& stream,
                                 ba::ControlNetwork const& cnet ) {
    stream << cnet.size() - cnet.num_ground_control_points() << "\n";
    BOOST_FOREACH( ba::ControlPoint const& cp, cnet ) {
      if ( cp.type() == ba::ControlPoint::GroundControlPoint )
        continue;
      stream << cp.position()[0] << " " << cp.position()[1] << " "
          << cp.position()[2] << " 0 0 0 " << cp.size();
      BOOST_FOREACH( ba::ControlMeasure const& cm, cp ) {
        stream << " " << cm.image_id() << " 0 "
               << cm.position()[0] << " " << cm.position()[1];
      }
      stream << "\n";
    }
  }

  // User writing functions
  template <class CameraT>
  void write_nvm_iterator_ptr( std::string const& file,
                               CameraT begin, CameraT end,
                               ba::ControlNetwork const& cnet ) {
    namespace fs = boost::filesystem;
    CameraT begin_copy = begin;
    size_t num_cameras = std::distance(begin_copy, end);

    std::ofstream nvm( fs::change_extension( file, ".nvm").string().c_str(),
                       std::ofstream::out );
    nvm << std::setprecision(12);
    nvm << "NVM_V3_R9T\n" << num_cameras << "\n";
    while ( begin != end ) {
      boost::shared_ptr<camera::PinholeModel> pin(*begin);
      write_nvm_camera( nvm, pin.get() );
      begin++;
    }
    write_nvm_controlnetwork( nvm, cnet );
  }

  template <class FocalT, class RotT, class TransT>
  void write_nvm_r9t( std::string const& file,
                      FocalT f_begin, FocalT f_end,
                      RotT rot_begin, RotT rot_end,
                      TransT trns_begin, TransT trns_end,
                      ba::ControlNetwork const& cnet ) {
    namespace fs = boost::filesystem;
    FocalT begin_copy  = f_begin;
    size_t num_cameras = std::distance(begin_copy, f_end);

    std::ofstream nvm( fs::change_extension( file, ".nvm").string().c_str(),
                       std::ofstream::out );
    nvm << std::setprecision(12);
    nvm << "NVM_V3_R9T\n" << num_cameras << "\n";
    while ( f_begin != f_end ) {
      write_nvm_r9t(nvm, *f_begin, *rot_begin,
                    *trns_begin );
      f_begin++;
      rot_begin++;
      trns_begin++;
    }
    write_nvm_controlnetwork( nvm, cnet );
  }

  // Reading functions
  camera::PinholeModel* read_nvm_camera(std::istream& stream) {
    double focal; std::string name; int buf;
    Vector3 translation;
    Matrix3x3 rotation;
    stream >> name >> focal
           >> rotation(0,0) >> rotation(0,1) >> rotation(0,2)
           >> rotation(1,0) >> rotation(1,1) >> rotation(1,2)
           >> rotation(2,0) >> rotation(2,1) >> rotation(2,2)
           >> translation[0] >> translation[1] >> translation[2]
           >> buf >> buf;
    Vector3 center = -transpose(rotation)*translation;
    return new camera::PinholeModel( center, transpose(rotation),
                                     focal, focal, 0, 0,
                                     Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1),
                                     camera::NullLensDistortion() );
  }
  template <class RotT, class TransT>
  void read_nvm_r9t(std::istream& stream,
                    float& focal,
                    RotT& rotation,
                    TransT& translation) {
    std::string name;
    int buf;
    stream >> name >> focal
           >> rotation(0,0) >> rotation(0,1) >> rotation(0,2)
           >> rotation(1,0) >> rotation(1,1) >> rotation(1,2)
           >> rotation(2,0) >> rotation(2,1) >> rotation(2,2)
           >> translation[0] >> translation[1] >> translation[2]
           >> buf >> buf;
  }
  void read_nvm_measurement(std::istream& stream,
                            ba::ControlPoint& cp) {
    float fbuf;
    size_t num_measurements;
    Vector3 position;
    stream >> position[0] >> position[1]
           >> position[2] >> fbuf >> fbuf >> fbuf >> num_measurements;
    cp.set_position(position);
    cp.resize( num_measurements );
    for ( size_t i = 0; i < num_measurements; i++ ) {
      size_t cam_id; Vector3f measure;
      stream >> cam_id >> measure[0] >> measure[1] >> measure[2];
      cp[i] = ba::ControlMeasure( measure[1], measure[2], 1, 1, cam_id );
    }
  }

  // User reading functions
  template <class CameraStructT>
  void read_nvm_iterator_ptr( std::string const& file,
                              CameraStructT& camera_structure,
                              ba::ControlNetwork& cnet ) {
    std::ifstream nvm( file.c_str(), std::ios::in );
    if (!nvm.is_open())
      vw_throw( ArgumentErr() << "Unable to open: " << file << "!\n" );
    std::string key;
    size_t num_cameras;
    nvm >> key >> num_cameras;

    // Allocate memory
    camera_structure.resize(num_cameras);
    typedef typename CameraStructT::iterator CIter;
    for ( CIter camera = camera_structure.begin();
          camera != camera_structure.end(); camera++ ) {
      camera->reset( read_nvm_camera(nvm) );
    }

    // Reading points
    size_t num_pts;
    nvm >> num_pts;
    cnet.resize( num_pts );
    BOOST_FOREACH( ba::ControlPoint& cp, cnet ) {
      read_nvm_measurement( nvm, cp );
    }
  }

  template <class FocalStructT, class RotStructT, class TStructT>
  void read_nvm_r9t( std::string const& file,
                     FocalStructT& focal_structure,
                     RotStructT& rot_structure,
                     TStructT& trans_structure,
                     ba::ControlNetwork& cnet ) {
    std::ifstream nvm( file.c_str(), std::ios::in );
    if (!nvm.is_open())
      vw_throw( ArgumentErr() << "Unable to open: " << file << "!\n" );
    std::string key;
    size_t num_cameras;
    nvm >> key >> num_cameras;

    // Allocating memory
    focal_structure.resize(num_cameras);
    rot_structure.resize(num_cameras);
    trans_structure.resize(num_cameras);

    // Define iterators
    typedef typename FocalStructT::iterator FIter;
    typedef typename RotStructT::iterator   RIter;
    typedef typename TStructT::iterator     TIter;
    FIter fbegin = focal_structure.begin();
    RIter rbegin = rot_structure.begin();
    TIter tbegin = trans_structure.begin();
    while ( fbegin != focal_structure.end() ) {
      read_nvm_r9t(nvm, *fbegin, *rbegin, *tbegin );
      fbegin++; rbegin++; tbegin++;
    }
    size_t num_pts;
    nvm >> num_pts;
    cnet.resize( num_pts );
    BOOST_FOREACH( ba::ControlPoint& cp, cnet ) {
      read_nvm_measurement( nvm, cp );
    }
  }
}

#endif//__NVM_IO_H__
