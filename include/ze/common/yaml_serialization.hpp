// THESE FUNCTIONS ARE FROM ETHZ-ASL ASLAM_CV_COMMON

#pragma once

#include <fstream>
#include <list>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <ze/common/logging.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {

template <typename T>
T extractChild(const YAML::Node& node, const std::string& key)
{
  if (!node.IsMap())
  {
    throw YAML::Exception(node.Mark(), "Node is not a map");
  }

  const YAML::Node child = node[key];
  if (!child)
  {
    throw YAML::Exception(node.Mark(), "key '" + key + "' does not exist");
  }

  return child.as<T>();
}

// yaml serialization helper function for the Eigen3 Matrix object.
// The matrix is a base class for dense matrices.
// http://eigen.tuxfamily.org/dox-devel/TutorialMatrixClass.html
template <class Scalar_, int A_, int B_, int C_, int D_, int E_>
struct convert<Eigen::Matrix<Scalar_, A_, B_, C_, D_, E_> >
{
  template <class Scalar, int A, int B, int C, int D, int E>
  static Node encode(const Eigen::Matrix<Scalar, A, B, C, D, E>& M)
  {
    Node node;
    typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
    IndexType rows = M.rows();
    IndexType cols = M.cols();
    node["rows"] = rows;
    node["cols"] = cols;
    CHECK_GT(rows, 0);
    CHECK_GT(cols, 0);
    for(IndexType i = 0; i < rows; ++i)
    {
      for(IndexType j = 0; j < cols; ++j)
      {
        node["data"].push_back(M(i, j));
      }
    }
    return node;
  }

  template <class Scalar, int A, int B, int C, int D, int E>
  static bool decode(const Node& node,
                     Eigen::Matrix<Scalar, A, B, C, D, E>& M)
  {
    if(!node.IsMap())
    {
      LOG(ERROR) << "Unable to get parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();

    if(rows != A || cols != B)
    {
      LOG(ERROR) << "The matrix is the wrong size (rows, cols). Wanted: (" << A << ","
          << B << "), got (" << rows << ", " << cols << ")";
      return false;
    }

    size_t expected_size = M.rows() * M.cols();
    if (!node["data"].IsSequence())
    {
      LOG(ERROR) << "The matrix data is not a sequence.";
      return false;
    }
    if(node["data"].size() != expected_size)
    {
      LOG(ERROR) << "The data sequence is the wrong size. Wanted: " << expected_size <<
          ", got: " << node["data"].size();
      return false;
    }

    YAML::const_iterator it = node["data"].begin();
    YAML::const_iterator it_end = node["data"].end();
    if(rows > 0 && cols > 0)
    {
      for(IndexType i = 0; i < rows; ++i)
      {
        for(IndexType j = 0; j < cols; ++j)
        {
          CHECK(it != it_end);
          M(i, j) = it->as<Scalar>();
          ++it;
        }
      }
    }
    return true;
  }

  template <class Scalar, int B, int C, int D, int E>
  static bool decode(const Node& node,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, B, C, D, E>& M)
  {
    if(!node.IsMap())
    {
      LOG(ERROR) << "Unable to get parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, B, C, D, E>::Index
        IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();
    CHECK_GE(rows, 0);
    CHECK_GE(cols, 0);

    if(cols != B)
    {
      LOG(ERROR) << "The matrix is the wrong size (rows, cols). Wanted: (" << rows << ","
          << B << "), got (" << rows << ", " << cols << ")";
      return false;
    }

    M.resize(rows, Eigen::NoChange);

    size_t expected_size = M.rows() * M.cols();
    if (!node["data"].IsSequence())
    {
      LOG(ERROR) << "The matrix data is not a sequence.";
      return false;
    }
    if(node["data"].size() != expected_size)
    {
      LOG(ERROR) << "The data sequence is the wrong size. Wanted: " << expected_size <<
          ", got: " << node["data"].size();
      return false;
    }

    YAML::const_iterator it = node["data"].begin();
    YAML::const_iterator it_end = node["data"].end();
    if(rows > 0 && cols > 0)
    {
      for(IndexType i = 0; i < rows; ++i)
      {
        for(IndexType j = 0; j < cols; ++j)
        {
          CHECK(it != it_end);
          M(i, j) = it->as<Scalar>();
          ++it;
        }
      }
    }
    return true;
  }

  template <class Scalar, int A, int C, int D, int E>
  static bool decode(const Node& node,
                     Eigen::Matrix<Scalar, A, Eigen::Dynamic, C, D, E>& M)
  {
    if(!node.IsMap())
    {
      LOG(ERROR) << "Unable to get parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, A, Eigen::Dynamic, C, D, E>::Index IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();
    CHECK_GE(rows, 0);
    CHECK_GE(cols, 0);

    if(rows != A)
    {
      LOG(ERROR) << "The matrix is the wrong size (rows, cols). Wanted: (" << A << ","
          << cols << "), got (" << rows << ", " << cols << ")";
      return false;
    }

    M.resize(Eigen::NoChange, cols);

    size_t expected_size = M.rows() * M.cols();
    if (!node["data"].IsSequence())
    {
      LOG(ERROR) << "The matrix data is not a sequence.";
      return false;
    }
    if(node["data"].size() != expected_size)
    {
      LOG(ERROR) << "The data sequence is the wrong size. Wanted: " << expected_size <<
          ", got: " << node["data"].size();
      return false;
    }

    YAML::const_iterator it = node["data"].begin();
    YAML::const_iterator it_end = node["data"].end();
    if (rows > 0 && cols > 0)
    {
      for (IndexType i = 0; i < rows; ++i)
      {
        for (IndexType j = 0; j < cols; ++j)
        {
          CHECK(it != it_end);
          M(i, j) = it->as<Scalar>();
          ++it;
        }
      }
    }
    return true;
  }

  template <class Scalar, int C, int D, int E>
  static bool decode(
      const Node& node,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, C, D, E>& M)
  {
    if(!node.IsMap())
    {
      LOG(ERROR) << "Unable to get parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, C, D,
                                   E>::Index IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();
    CHECK_GE(rows, 0);
    CHECK_GE(cols, 0);

    M.resize(rows, cols);

    size_t expected_size = M.rows() * M.cols();
    if (!node["data"].IsSequence())
    {
      LOG(ERROR) << "The matrix data is not a sequence.";
      return false;
    }
    if(node["data"].size() != expected_size)
    {
      LOG(ERROR) << "The data sequence is the wrong size. Wanted: " << expected_size <<
          ", got: " << node["data"].size();
      return false;
    }
    YAML::const_iterator it = node["data"].begin();
    YAML::const_iterator it_end = node["data"].end();
    if(rows > 0 && cols > 0)
    {
      for(IndexType i = 0; i < rows; ++i)
      {
        for(IndexType j = 0; j < cols; ++j)
        {
          CHECK(it != it_end);
          M(i, j) = it->as<Scalar>();
          ++it;
        }
      }
    }
    return true;
  }
};

}  // namespace YAML
