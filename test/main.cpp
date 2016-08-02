#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <ze/common/transformation.h>
#include <ze/common/types.h>

using namespace cv;

int main (int argc, char** argv)
{
  Mat left_img = imread("/opt/matlab/toolbox/vision/visiondata/calibration/stereo/left/left01.png", CV_LOAD_IMAGE_GRAYSCALE);
  Mat right_img = imread("/opt/matlab/toolbox/vision/visiondata/calibration/stereo/right/right01.png", CV_LOAD_IMAGE_GRAYSCALE);

  Mat_<double> left_K(3, 3);
  left_K << 1038.30795208136, 0.0, 656.603479450933, 0.0, 1038.27460619720, 485.636298956628, 0.0, 0.0, 1.0;
  Mat_<double> right_K(3, 3);
  right_K << 1042.92459263593, 0.0, 640.518721869622, 0.0, 1042.70387431383, 478.875664608114, 0.0, 0.0, 1.0;

  Mat_<double> left_dist(1, 4);
  left_dist << -0.360861731425732, 0.182500063498877, 0.0, 0.0;
  Mat_<double> right_dist(1, 4);
  right_dist << -0.361712285838908, 0.183985883045769, 0.0, 0.0;

  Mat_<double> R_l_r(3, 3);
  R_l_r << 0.999984670393789, -0.000162531377231556, -0.00553466900343020,
      0.000140913190843947, 0.999992361139114f, -0.00390612428012933,
      0.00553526159262252, 0.00390528449291277, 0.999977054552819;

  Mat_<double> t_l_r(3, 1);
  t_l_r << -119.866795432659, -0.341905598869513, 0.177782478039805;

  Mat left_H;
  Mat right_H;
  Mat left_P;
  Mat right_P;
  Mat disp_to_depth_Q;
  Size img_size = left_img.size();

  stereoRectify(left_K, left_dist,
                right_K, right_dist,
                img_size,
                R_l_r, t_l_r,
                left_H, right_H,
                left_P, right_P,
                disp_to_depth_Q,
                CALIB_ZERO_DISPARITY,
                0, img_size);

  std::cout << "stereo rectify new left P:" << std::endl
            << left_P << std::endl;
  std::cout << "stereo rectify new right P:" << std::endl
            << right_P << std::endl;

  Mat left_map_x;
  Mat left_map_y;
  Mat right_map_x;
  Mat right_map_y;

  initUndistortRectifyMap(left_K, left_dist,
                          left_H, left_P,
                          img_size,
                          CV_16SC2,
                          left_map_x, left_map_y);
  initUndistortRectifyMap(right_K, right_dist,
                          right_H, right_P,
                          img_size,
                          CV_16SC2,
                          right_map_x, right_map_y);

  Mat rectified_left;
  Mat rectified_right;

  remap(left_img, rectified_left, left_map_x, left_map_y, INTER_LINEAR);
  remap(right_img, rectified_right, right_map_x, right_map_y, INTER_LINEAR);

  imshow("LEFT", left_img);
  imshow("RIGHT", right_img);
  imshow("RECT_LEFT", rectified_left);
  imshow("RECT_RIGHT", rectified_right);
  waitKey();

  std::cout << "init rectify new left P:" << std::endl
            << left_P << std::endl;
  std::cout << "init rectify new right P:" << std::endl
            << right_P << std::endl;

  std::cout << "left H: " << std::endl
            << left_H << std::endl;
  std::cout << "right H: " << std::endl
            << right_H << std::endl;

  std::cout << "new image size: " << std::endl
            << img_size << std::endl;

  return 0;
}
