#ifndef AUTO_TRAX_IMAGE_PROCESSING_H
#define AUTO_TRAX_IMAGE_PROCESSING_H

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace auto_trax {
// Default camera intrinsic parameter values
static const double kDefaultFx = 538.82572;
static const double kDefaultFy = 538.46549;
static const double kDefaultCx = 317.85920;
static const double kDefaultCy = 239.92291;
static const double kDefaultD0 = 0.018918;
static const double kDefaultD1 = -0.085798;
static const double kDefaultD2 = 0.001551;
static const double kDefaultD3 = 0.000980;

// Default segmentation parameter values
static const int kDefaultHorizonPixels = 200;

// Constants
static const cv::Scalar kBlack(0.0, 0.0, 0.0);
static const cv::Scalar kWhite(255.0, 255.0, 255.0);
static const int kPolygonPoints = 4;
static const int kFrameWidth = 640;
static const int kFrameHeight = 480;

struct CameraIntrinsicParameters {
  CameraIntrinsicParameters():
      f_x_(kDefaultFx),
      f_y_(kDefaultFy),
      c_x_(kDefaultCx),
      c_y_(kDefaultCy),
      d_0_(kDefaultD0),
      d_1_(kDefaultD1),
      d_2_(kDefaultD2),
      d_3_(kDefaultD3) {
  }

  // Focal lengths
  double f_x_;
  double f_y_;

  // Principal point pixel coordinates
  double c_x_;
  double c_y_;

  // Distortion coefficients
  double d_0_;
  double d_1_;
  double d_2_;
  double d_3_;
};

struct SegmentationParameters {
  SegmentationParameters():
      horizon_pixels_(kDefaultHorizonPixels) {
  }

  // Parameters for segmenting out the track lines
  int horizon_pixels_;
};

class ImageProcessing {
  public:
    ImageProcessing();
    virtual ~ImageProcessing();

    inline void Project3DPtToPixels(const Eigen::Vector4d pt_3d, Eigen::Vector3d &pixels, double scale) {
      pixels = (camera_matrix_ * transform_matrix_ * pt_3d * (1.0 / scale));
    }

    inline void ProjectPixelsTo3D(const Eigen::Vector3d pixels, Eigen::Vector4d &pt_3d, double scale) {
      pt_3d = transform_matrix_.transpose() * camera_matrix_.inverse() * scale * pixels;
    }

    void SegmentTracks(const cv::Mat& img_in, cv::Mat& img_out);

    void UpdateParameters();

    CameraIntrinsicParameters camera_intrinsics_;

    SegmentationParameters seg_params_;

  private:
    Eigen::Matrix3d camera_matrix_;

    Eigen::MatrixXd transform_matrix_;
};
}

#endif // AUTO_TRAX_IMAGE_PROCESSING_H
