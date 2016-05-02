#include <ze/ros/tf_bridge.h>

namespace ze {

tf::Transform transformationToTF(const Transformation& ze_T)
{
  tf::Transform tf_T;
  tf_T.setOrigin(
        tf::Vector3(
          ze_T.getPosition().x(),
          ze_T.getPosition().y(),
          ze_T.getPosition().z()));
  tf_T.setRotation(
        tf::Quaternion(
          ze_T.getRotation().x(),
          ze_T.getRotation().y(),
          ze_T.getRotation().z(),
          ze_T.getRotation().w()));
  return tf_T;
}

} // ze namespace
