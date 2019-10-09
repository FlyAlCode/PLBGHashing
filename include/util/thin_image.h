#ifndef THIN_IMAGE_H_
#define THIN_IMAGE_H_

#include <opencv2/core/core.hpp>

void FillImageHole(const cv::Mat &input_img, cv::Mat &output_img);

void ThinImage(const cv::Mat &src,
               cv::Mat &thined_img,
               int binary_threshold = 30,
               const int maxIterations = -1);

#endif