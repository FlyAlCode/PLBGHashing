#ifndef LINE_SEGMENT_H_
#define LINE_SEGMENT_H_

#include <vector>
#include <iostream>
#include <memory>

#include <opencv2/core/core.hpp>

//#include "keypoint.h"


namespace graph_creator{
class KeyPoint;
typedef std::shared_ptr<KeyPoint> KeyPointPtr;
    
class LineSegment{
 public:
    LineSegment();
    ~LineSegment();
    
    /*  @set the start point, and ID as well
     *  @input:
     *  @ start --- the start point
     */
    void Init(const KeyPointPtr &start);
    
    /* @add a new point to the line segment, and update the property of it(eg. length) at the same time
     * @input:
     *    point---new point to add to the line segment  
     */
    void AddPoint(const cv::Point2f & point);
    
    /*  @add all points in the area to the line segment
     *  @input:
     *  @ area --- the given area
     */
    void AddRectArea(const cv::Rect &area);
    
    /*  @set endpoint(start/end) for the line segment
     *  @input:
     *  @ endpoint --- point to Set
     */
    void SetEndPoint(const KeyPointPtr &endpoint);
    
    /*  @replace the old endpoint with the new_point,return false when old_point not found
     */
    bool RefineEndPoint(const KeyPointPtr &new_point, const KeyPointPtr &old_point);
    
    /* @calculate new linearity of the line segment when points change
     */
    void UpdateLinearity();

    cv::Point2f get_start_pt() const;
    cv::Point2f get_end_pt() const;

    inline int get_id() const {return id_;}
    
    inline std::vector<cv::Point2f>  get_points() const{
        return points_;
    }


private:
    std::vector<cv::Point2f> points_;                   // points belong to the line segment
    cv::Point2f start_point_;                           // the start point of the line segment
    cv::Point2f end_point_;                             // end point of the line segment
    cv::Point2f middle_point_;                          // the middle point of the line segment
    cv::Point2f direction_vector_;                      // the direction of the line segment
    double linearity_;                                  // indicate whether the line segment is a line or a curve
    double length_;                                     // the number of pixel of the line segment
    
//     KeyPointPtr start_keypoint_;                        // the start keypoint
//     KeyPointPtr end_keypoint_;                          // the end keypoint
    std::vector<KeyPointPtr> endpoints_;                 // endpoints for the line
//    cv::Point2f current_grow_direction_;                 // the direction to search for next point
//    cv::Point2f next_grow_point_;                        // next point to anailsys
    
    int id_;                                             // the unique id for the point
    static int id_count_;                                // used to calculate id
};


typedef std::shared_ptr<LineSegment> LineSegmentPtr;
}  //namespace graph_creator




#endif


