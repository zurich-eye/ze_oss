#include <string>
#include <utility>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>

#include <ze/cameras/camera.h>
#include <ze/cameras/camera_yaml_serialization.h>

namespace ze {

template<typename Scalar>
Camera<Scalar>::Camera(const int width, const int height)
  : width_(width)
  , height_(height)
{}

template<typename Scalar>
typename Camera<Scalar>::Ptr Camera<Scalar>::loadFromYaml(
    const std::string& yaml_file)
{
  try {
    YAML::Node doc = YAML::LoadFile(yaml_file.c_str());
    return doc.as<Camera::Ptr>();
  } catch (const std::exception& ex) {
    LOG(ERROR) << "Failed to load Camera from file " << yaml_file << " with the error: \n"
               << ex.what();
  }
  // Return nullptr in the failure case.
  return Camera<Scalar>::Ptr();
}

template<typename Scalar>
void Camera<Scalar>::setMask(const cv::Mat& mask) {
  CHECK_EQ(height_, mask.rows);
  CHECK_EQ(width_, mask.cols);
  CHECK_EQ(mask.type(), CV_8UC1);
  mask_ = mask;
}

template<typename Scalar>
void Camera<Scalar>::loadMask(const std::string& mask_file)
{
  cv::Mat mask(cv::imread(mask_file, 0));
  if(mask.data)
    setMask(mask);
  else
    LOG(FATAL) << "Unable to load mask file.";
}

template<typename Scalar>
bool Camera<Scalar>::isMasked(
    const Eigen::Ref<const Vector2>& keypoint) const
{
  return keypoint[0] < 0.0 ||
         keypoint[0] >= static_cast<double>(width_) ||
         keypoint[1] < 0.0 ||
         keypoint[1] >= static_cast<double>(height_) ||
         (!mask_.empty() &&
           mask_.at<uint8_t>(static_cast<int>(keypoint[1]), static_cast<int>(keypoint[0])) == 0);
}

} // namespace ze
