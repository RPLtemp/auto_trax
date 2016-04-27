#ifndef AUTO_TRAX_IMAGE_PROCESSING_H
#define AUTO_TRAX_IMAGE_PROCESSING_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace auto_trax {
// Default values
static const int kDefaultHorizonPixels = 200;
static const int kDefaultRThresh = 255;
static const int kDefaultGThresh = 230;
static const int kDefaultBThresh = 160;
static const int kDefaultRGBRange = 10;

// Constants
static const cv::Scalar kBlack(0.0, 0.0, 0.0);
static const cv::Scalar kWhite(255.0, 255.0, 255.0);
static const int kPolygonPoints = 4;

struct ImageProcessingParameters {
  ImageProcessingParameters():
      horizon_pixels_(kDefaultHorizonPixels),
      r_thresh_(kDefaultRThresh),
      g_thresh_(kDefaultGThresh),
      b_thresh_(kDefaultBThresh),
      rgb_range_(kDefaultRGBRange) {
    lower_bound_ = cv::Scalar(b_thresh_ - rgb_range_, g_thresh_ - rgb_range_, r_thresh_ - rgb_range_);
    upper_bound_ = cv::Scalar(b_thresh_ + rgb_range_, g_thresh_ + rgb_range_, r_thresh_ + rgb_range_);
  }

  int horizon_pixels_;
  int r_thresh_;
  int g_thresh_;
  int b_thresh_;
  int rgb_range_;

  cv::Scalar lower_bound_;
  cv::Scalar upper_bound_;
};

class ImageProcessing {
  public:
    ImageProcessing();
    virtual ~ImageProcessing();

    void UpdateDerivedParameters(ImageProcessingParameters& params);

    void SegmentByColoredTracks(const cv::Mat& img_in, cv::Mat& img_out);

    ImageProcessingParameters params_;
};
}

#endif // AUTO_TRAX_IMAGE_PROCESSING_H
