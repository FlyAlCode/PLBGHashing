#include "image_to_vector.h"
#include "keypoint.h"
#include "line_segment.h"
#include "cross_points_detector.h"
#include "graph_creator.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "util/thin_image.h"

// debug
#include <opencv2/highgui/highgui.hpp>

namespace graph_creator {

void ImageToVector::Run(const cv::Mat &img,
                        int min_cross_pt_distance,
                        float min_node_distance,
                        ghbgl::PolyLineSet &polyline_set,
                        bool thin_img) {
    graph_creator::CrossPointDetector cross_pt_detector;     
    cv::Mat neighbor_img;
    cv::Mat img_tmp;
    std::vector<cv::Rect> center_areas;
    if(img.channels()==3)
        cv::cvtColor(img, img_tmp, CV_RGB2GRAY);
    else
        img.copyTo(img_tmp);
    // cv::threshold(img_tmp, img_tmp, 30, 1, CV_THRESH_BINARY);
    ThinImage(img_tmp, img_tmp);

    
    // debug
    // cv::imshow("binary image", img_tmp * 255);
    // cv::waitKey();

    cross_pt_detector.Run(img_tmp, neighbor_img, center_areas);
    
    GraphCreator graph_creator;
    std::map<int, LineSegmentPtr> line_segment_set;
    std::map<int, KeyPointPtr> keypoint_set;
    graph_creator.Run(neighbor_img, center_areas, min_cross_pt_distance, line_segment_set, keypoint_set);

    // 提取所有的线段，并计算线段初始位置和终止位置附近的线性度
    polyline_set.lines_.resize(line_segment_set.size());
    polyline_set.start_pts_linearity_.resize(line_segment_set.size());
    polyline_set.end_pts_linearity_.resize(line_segment_set.size());
    int i = 0;
    for (auto iter = line_segment_set.begin(); iter != line_segment_set.end(); iter++)  {
        std::vector<cv::Point2f> points = iter->second->get_points();
        PointSetToPolyLine(points,
                           iter->second->get_start_pt(),
                           iter->second->get_end_pt(),
                           min_node_distance,
                           polyline_set.lines_[i],
                           polyline_set.start_pts_linearity_[i],
                           polyline_set.end_pts_linearity_[i]);
        ++i;
    }
}


void ImageToVector::PointSetToPolyLine(const std::vector<cv::Point2f> &middle_pts,   
                            const cv::Point2f &start_pt,
                            const cv::Point2f &end_pt,            
                            float node_min_distance,                             
                            std::vector<cv::Point2f> &polyline,                  
                            float &start_pt_linearity,                          
                            float &end_pt_linearity){
    polyline.clear();

    int current_pt_index = 0;
    int last_second_node_pt_index = 0;
    int first_second_node_pt_index = 0;
    int last_third_node_pt_index = 0;
    // 是否存在交叉点作为起点
    // 不存在
    if (start_pt.x == -1 && start_pt.y == -1){
        polyline.push_back(middle_pts[0]);
        start_pt_linearity = -1;
        current_pt_index = 1;
    }
    else{
        polyline.push_back(start_pt);
    }

    // 中间段插值
    float d_tmp = cv::norm(polyline.back()-middle_pts[current_pt_index]);
    for (int i = current_pt_index + 1; i < middle_pts.size(); i++) {
        d_tmp += cv::norm(middle_pts[i] - middle_pts[i - 1]);
        if(d_tmp>node_min_distance){
            polyline.push_back(middle_pts[i]);
            d_tmp = 0;
            
            if(last_second_node_pt_index!=0){
                last_third_node_pt_index = last_second_node_pt_index;
            }
            last_second_node_pt_index = i;

            if(first_second_node_pt_index==0){
                first_second_node_pt_index = i;
            }
        }
    }

    if (first_second_node_pt_index == 0){                       // 线段过短，
        if (end_pt.x == -1 && end_pt.y == -1){
            polyline.push_back(middle_pts.back());
        }
        else{
            polyline.push_back(end_pt);
        }
        start_pt_linearity = -1;                                // 线段过短，不作为基底的选择对象
        end_pt_linearity = -1;
        return;
    }
    else{
        // 计算起点的线性度
        if (start_pt_linearity != -1) {
            start_pt_linearity = CalLinearity(start_pt, polyline[1], middle_pts, 0, first_second_node_pt_index);
        }
    }
    
    // 是否存在交叉点作为最后一个点
    if (end_pt.x == -1 && end_pt.y == -1){
        end_pt_linearity = -1;
    }
    else{
        // 处理中间仅仅插入一个点的情况
        if(first_second_node_pt_index==last_second_node_pt_index){
            if (cv::norm(end_pt - polyline.back()) < node_min_distance){
                polyline.pop_back();
                polyline.push_back(end_pt);
                end_pt_linearity = CalLinearity(polyline[0], end_pt, middle_pts, 0, middle_pts.size() - 1);
            }
            else{
                polyline.push_back(end_pt);
                end_pt_linearity = CalLinearity(middle_pts[last_second_node_pt_index], end_pt, middle_pts, last_second_node_pt_index + 1, middle_pts.size() - 1);
            }
        }
        else{
            // 判断最后一个点是否距离end_pt太近
            if (cv::norm(end_pt - polyline.back()) < node_min_distance){
                polyline.pop_back();
                polyline.push_back(end_pt);
                end_pt_linearity = CalLinearity(middle_pts[last_third_node_pt_index], end_pt, middle_pts, last_third_node_pt_index + 1, middle_pts.size() - 1);
            }
            else{
                polyline.push_back(end_pt);
                end_pt_linearity = CalLinearity(middle_pts[last_second_node_pt_index], end_pt, middle_pts, last_second_node_pt_index + 1, middle_pts.size() - 1);
            }
        }        
    }
}

float ImageToVector::CalLinearity(const cv::Point2f &start_pt,
                   const cv::Point2f &end_pt,
                   const std::vector<cv::Point2f> &middle_pts,
                   int middle_first_pt_index,
                   int middle_last_pt_index){
    // (y0-y1)x+(x1-x0)y+x0y1-x1y0=0
    float a = start_pt.y - end_pt.y;
    float b = end_pt.x - start_pt.x;
    float c = start_pt.x * end_pt.y - end_pt.x * start_pt.y;

    float d = sqrt(a * a + b * b);
    float linearity = 0;
    for (int i = middle_first_pt_index; i <= middle_last_pt_index; i++){
        float d = a * middle_pts[i].x + b * middle_pts[i].y + c;
        if(d<0)
            d = -d;
        linearity += d;
    }
    linearity /= d;
    linearity /= (middle_last_pt_index - middle_first_pt_index + 1);
    return linearity;
}

} // namespace graph_creator 