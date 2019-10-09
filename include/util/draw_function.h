#ifndef UTILS_DRAW_FUNCTION_H_
#define UTILS_DRAW_FUNCTION_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

bool DrawMergeOnMap(const cv::Mat &query_img,               
               const cv::Mat &map_img,                            
               const cv::Mat &H, 
               int padding_size,                        
               cv::Mat &draw_img);

bool DrawRoadOnSat(const cv::Mat &query_img,                    // the road image(0 means there exist no roads)
                   const cv::Mat &map_img,                      // the reference sat image
                   const cv::Mat &H,                            // transform pts in query image to map image
                   cv::Mat &draw_img);

// 给定两张图，及其对应的点对应，画出这些点的对应关系
void ShowMatch(const cv::Mat &query_img, const cv::Mat &ref_img,
               const std::vector<cv::Point2f> &query_pts,
               const std::vector<cv::Point2f> &ref_pts,
               cv::Mat &draw_img);
#endif