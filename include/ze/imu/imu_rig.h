#pragma once

#include <string>
#include <vector>

#include <ze/imu/imu_model.h>
#include <ze/common/macros.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>

namespace ze {

using ImuVector = std::vector<ImuModel::Ptr>;

class ImuRig
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(ImuRig);

  ImuRig() = delete;

  ImuRig(
      const TransformationVector& T_B_S,
      const ImuVector& imus,
      const std::string& label);

  //! Load a imu rig form a yaml file. Returns a nullptr if the loading fails.
  static ImuRig::Ptr loadFromYaml(const std::string& yaml_file);

  //! @name Imu poses with respect to body frame.
  //! @{
  inline const Transformation& T_S_B(size_t imu_index) const
  {
    DEBUG_CHECK_LT(imu_index, T_S_B_.size());
    return T_S_B_[imu_index];
  }

  inline const TransformationVector& T_S_B_vec() const
  {
    return T_S_B_;
  }

  inline const Transformation& T_B_S(size_t imu_index) const
  {
    DEBUG_CHECK_LT(imu_index, T_B_S_.size());
    return T_B_S_[imu_index];
  }

  inline const TransformationVector& T_B_S_vec() const
  {
    return T_B_S_;
  }
  //! @}

  //! @name Imu accessors.
  //! @{
  inline const ImuModel& at(size_t imu_index) const
  {
    return *imus_.at(imu_index);
  }

  inline ImuModel::Ptr atShared(size_t imu_index)
  {
    return imus_.at(imu_index);
  }

  inline std::shared_ptr<const ImuModel> atShared(size_t imu_index) const
  {
    return imus_.at(imu_index);
  }

  inline const ImuVector& imus() const { return imus_; }
  //! @}

  inline size_t size() const { return imus_.size(); }

  inline const std::string& label() const { return label_; }

  //! @name Imu iteration.
  //! @{
  typedef ImuVector::value_type value_type;
  typedef ImuVector::iterator iterator;
  typedef ImuVector::const_iterator const_iterator;
  ImuVector::iterator begin() { return imus_.begin(); }
  ImuVector::iterator end() { return imus_.end(); }
  ImuVector::const_iterator begin() const { return imus_.begin(); }
  ImuVector::const_iterator end() const { return imus_.end(); }
  ImuVector::const_iterator cbegin() const { return imus_.cbegin(); }
  ImuVector::const_iterator cend() const { return imus_.cend(); }
  //! @}

private:
  //! The mounting transformations.
  TransformationVector T_S_B_;
  TransformationVector T_B_S_;

  //! The imu models.
  ImuVector imus_;

  //! The rig label
  std::string label_;

};

} // namespace ze

