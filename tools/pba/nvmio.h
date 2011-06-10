#ifndef __NVM_IO_H__
#define __NVM_IO_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

namespace vw {

  // Writing functions
  void write_nvm_camera(std::ostream& stream,
                        camera::CameraModel const* cam) {
  }
  template <class RotT, class TransT>
  void write_nvm_r9t(std::ostream& stream,
                     float const& focal,
                     RotT const& rotation,
                     TransT const& translation) {
  }
  void write_nvm_measurement(std::ostream& stream,
                             ba::ControlPoint const& cp) {
  }

  // User writing functions
  template <class CameraT>
  void write_nvm_iterator_ptr( std::string const& file,
                               CameraT begin, CameraT end,
                               ba::ControlNetwork const& cnet ) {
  }

  template <class FocalT, class RotT, class TransT>
  void write_nvm_r9t( std::string const& file,
                      FocalT f_begin, FocalT f_end,
                      RotT rot_begin, RotT rot_end,
                      TransT trns_begin, TransT trns_end,
                      ba::ControlNetwork const& cnet ) {
  }

  // Reading functions
  camera::CameraModel* read_nvm_camera(std::istream& stream) {
  }
  template <class RotT, class TransT>
  void read_nvm_9t(std::istream& stream,
                   float& focal,
                   RotT& rotation,
                   TransT& translation) {
  }
  void read_nvm_measurement(std::istream& stream,
                            ba::ControlPoint& cp) {
  }

  // User reading functions
  template <class CameraStructT>
  void read_nvm_iterator_ptr( std::string const& file,
                              CameraStructT& camera_structure,
                              ba::ControlNetwork& cnet ) {
  }

  template <class FocalStructT, class RotStructT, class TStructT>
  void read_nvm_r9t( std::string const& file,
                     FocalStructT& focal_structure,
                     RotStructT& rot_structure,
                     TStructT& trans_structure,
                     ba::ControlNetwork const& cnet ) {
  }
}

#endif//__NVM_IO_H__
