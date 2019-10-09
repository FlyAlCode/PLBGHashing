#ifndef GHBGL_IMAGE_TO_VECTOR_H_
#define GHBGL_IMAGE_TO_VECTOR_H_

#include <opencv2/core/core.hpp>
#include "basic_class.h"

namespace graph_creator {

class ImageToVector{
public:
    void Run(const cv::Mat &img,                        // 输入图像（可为彩色图像）
             int min_cross_pt_distance,                 // 两个交点最近的距离
             float min_node_distance,
             ghbgl::PolyLineSet &polyline_set,          // 提取的折线的集合
             bool thin_img = true);

private:
    void PointSetToPolyLine(const std::vector<cv::Point2f> &src_pts,                // 输入的点集(不包括起点和终点)
                            const cv::Point2f &start_pt,
                            const cv::Point2f &end_pt, 
                            float node_min_distance,                                // 折线两个节点的最小距离(像素距离)，用于对弧线降采样
                            std::vector<cv::Point2f> &polyline,                     // 得到的折线
                            float &start_pt_linearity,                              // 起始点附近点的线性程度
                            float &end_pt_linearity);                               // 终点附近点的线性度

    float CalLinearity(const cv::Point2f &start_pt,
                   const cv::Point2f &end_pt,
                   const std::vector<cv::Point2f> &middle_pts,
                   int middle_first_pt_index,
                   int middle_last_pt_index);
};

    

} // namespace graph_creator 

#endif