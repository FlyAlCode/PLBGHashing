#ifndef GRAPH_CREATOR_H_
#define GRAPH_CREATOR_H_
#include <map>
#include <vector>
#include <memory>
#include <iostream>

#include <opencv2/core/core.hpp>
// #include "keypoint.h"
// #include "line_segment.h"

namespace graph_creator{

class KeyPoint;
class LineSegment;
typedef std::shared_ptr<KeyPoint> KeyPointPtr;
typedef std::shared_ptr<LineSegment> LineSegmentPtr;

class GraphCreator{
public:
    enum GrowDirection{ UP=0, UP_LEFT=1, LEFT=2, DOWN_LEFT=3,
                        DOWN=4, DOWN_RIGHT=5, RIGHT=6, UP_RIGHT=7  };   // direction to search for new points
    GraphCreator();
    ~GraphCreator();
    
    /* @do some initilization work: init the flag_map_, 
     * @input:
     *      min_line_length --- used to set parametre min_line_length_
     */
    void Init(const int &min_line_length);
    
    /* @run all steps to transform binary image to graph
     * @input:
     *      neighbor_img--neighbor image 
     *      keypoint_area --- where the keypoint is
     *      min_line_length --- minimum length for a line to be treated as a line
     *  @output:
     *  @ line_segment_set --- all the line segments
     *  @ keypoint_set --- all the keypoints
     */
    void Run(const cv::Mat &neighbor_img, 
             const std::vector<cv::Rect> &keypoint_area, 
             const int &min_line_length,
             std::map<int, LineSegmentPtr> &line_segment_set,
             std::map<int, KeyPointPtr> &keypoint_set );
    
    /***************************************
     *         used just for debug         *
     * *************************************/
    void DrowKeypointAroundPoints(const cv::Mat &neighbor_img,
                                                          const std::vector<cv::Rect> &keypoint_area);
    
    void DrawRect(const cv::Rect &rect, const cv::Vec3b &color, cv::Mat &draw_img);
    
private:
    /* @create keypoint objects for all cross points(endpoint), and recored them in keypoint_set_
     * @and encode them, all IDS are recoreded in a Mat--flag_map_(waste memory, but save time)
     * @input:
     *      keypoint_area --- location for keypoints
     */
    void CreateAllCrossPoints(const std::vector<cv::Rect> &keypoint_area);

    /*  @used to provide start points for line segment expansion algrithom
     *  @input:
     *  @ kp --- the keypoint to calculate
     *  @output:
     *  @ start_rects --- start areas for line segment expansion
     */
    void FindAllStartRectsAroundKeypoint(const KeyPointPtr &kp, std::vector<cv::Rect> &start_rects);
    
    /*  @search around the centre points to find next points to expand
     *  @input:
     *  @ neighbor_img --- neighbor image after merging adjacent keypoints
     *  @ centre --- points area to search around
     *  @output:
     *  @ next_rect --- next points to expand
     */
    int ExpandToNextPoint(const cv::Rect &centre, const cv::Rect &last_centre, cv::Rect &next_rect);
    
    /*  @expend from start_point to form a line segment, and push back the line segment into 
     *  @line_segment_set_
     *  @input:
     *  @ start_point --- the start_point for the expension
     *  @ start_rect --- one of the connected rect area of the keypoint
     *  @return value:
     *  @ if the line has been created, return false; else, return true.
     */
    bool ExpandOneLineSegment(const KeyPointPtr &start_point, const cv::Rect &start_rect, LineSegmentPtr &line);
    
    /*  @If a line segment is short than min_line_length, than it is treat as a whole keypoint. 
     *  @The line segment will be removed, and the two endpoints of the line is merged into one;
     *  @lines connected to the old keypoints will be redefine---the endpoints of them point to 
     *  @new point objects
     *  @input:
     *  @ line---the line to be removed
     */
    void RemoveShortLine(const LineSegmentPtr &line);
    
    /*  @expand all line segments to form a whole graph 
     */
    void ExpandAllLineSegments();
    
    
//     std::vector<LineSegmentPtr> line_segment_stack_;                    // container for unsearched line
    std::map<int, LineSegmentPtr> line_segment_set_;                    // container for searched line
    std::map<int, KeyPointPtr> keypoint_set_;                           // container for keypoint
    LineSegmentPtr current_line_;                                       // current active line
    int min_line_length_;                                               // the minimum length(pixel) for a line segment to be called a line
    
    
    
//     cv::Mat flag_map_;                                                  // used to indicate the state for all point in map
    cv::Mat id_map_;                                                    // used to recored the ID for every keypoint, 
    cv::Mat visited_map_;                                               // used to recored the visited stated of points
    cv::Mat neighbor_img_;                                              // set by run 
};

}  // namespace graph_creator

#endif
