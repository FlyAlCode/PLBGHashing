#include "keypoint.h"
#include "line_segment.h"

namespace graph_creator{
    
int KeyPoint::id_count_ = 1;   

KeyPoint::KeyPoint(){
    id_ = id_count_;
    ++id_count_;
}

KeyPoint::~KeyPoint(){
    
}

void KeyPoint::Init(const cv::Rect &area, const int& type){
    point_type_ = type;
    point_area_ = area;
    connected_lines_.clear();
}

void KeyPoint::AddConnectedLine(const LineSegmentPtr& line){
    connected_lines_.push_back(line);
}



}  //namespace graph_creator
