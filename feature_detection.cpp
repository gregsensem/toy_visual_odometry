#include "opencv2/features2d/features2d.hpp"

void FastFeatureDetection(cv::Mat img_1, std::vector<cv::Point2f>& points1)
{
  std::vector<cv::KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool nonmaxSuppression = true;
  cv::FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  cv::KeyPoint::convert(keypoints_1, points1, std::vector<int>());
}

void GoodFeatureDetection(cv::Mat img, std::vector<cv::Point2f>& points)
{
    int MaxCorners = 2000;
    double QualityLevel = 0.01;
    double MinDistance = 10;
    cv::goodFeaturesToTrack(img,points,MaxCorners,QualityLevel,MinDistance);
}
