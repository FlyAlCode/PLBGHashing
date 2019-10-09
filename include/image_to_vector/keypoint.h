#ifndef _KEYPOINT_H_
#define KEYPOINT_H_

#include <list>
#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
//#include "line_segment.h"



namespace graph_creator{
    
class LineSegment;
typedef std::shared_ptr<LineSegment> LineSegmentPtr;
    
class KeyPoint{
public:
    //enum PointType{TERMINAL=1, THREE_CROSS=3, FOUR_CROSS=4, FIVE_CROSS=5};
    
    KeyPoint();
    ~KeyPoint();
    
    /* @init the point
     * @input: 
     *      point--the coordinate of the point
     *      type--the type for the point
     */
    void Init(const cv::Rect &area, const int &type);
    
    /* @add line connected to current point
     * @input:
     *      line--line to add 
     */
    void AddConnectedLine(const LineSegmentPtr &line);
    
//     inline cv::Point2f GetCoordinate() const {
//         return coordinate_;
//     }
    inline int get_id() const {
        return id_;
    } 
    
    inline cv::Rect get_area() const{
        return point_area_;
    }
    
    inline int get_type() const{
        return point_type_;
    }

    inline const cv::Point2f get_center() const { 
        return cv::Point2f(point_area_.x + point_area_.width / 2.0, point_area_.y + point_area_.height / 2.0); 
    }

    inline const int & type() const { return point_type_; }

private:
    cv::Rect point_area_;                                       // area for the KeyPoint
    std::vector<LineSegmentPtr> connected_lines_;                 // line segments connected to this point  
    int id_;                                                    // unique id for the point
    int point_type_;                                            // how many directions 
    
    static int id_count_;                                       // used to calculate id
};

                              
typedef std::shared_ptr<KeyPoint> KeyPointPtr;

}  //namespace graph_creator

#endif
