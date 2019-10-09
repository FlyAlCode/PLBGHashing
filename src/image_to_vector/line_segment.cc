#include "line_segment.h"
#include "keypoint.h"

namespace graph_creator{
    
int LineSegment::id_count_ = 2;
    
LineSegment::LineSegment(){
    id_ = id_count_;
    ++id_count_;
}

LineSegment::~LineSegment(){
    
}


void LineSegment::Init(const graph_creator::KeyPointPtr& start){
    endpoints_.clear();
    endpoints_.push_back(start);
    if(start!=nullptr)
        start_point_ = start->get_center();
}

void LineSegment::AddPoint(const cv::Point2f& point){
    points_.push_back(point);
}

void LineSegment::SetEndPoint(const graph_creator::KeyPointPtr& endpoint){
    endpoints_.push_back(endpoint);
    if(nullptr!=endpoint)
        end_point_ = endpoint->get_center();
}

bool LineSegment::RefineEndPoint(const graph_creator::KeyPointPtr& new_point, const graph_creator::KeyPointPtr& old_point){
    for(int i=0; i<endpoints_.size(); i++){
        if(endpoints_[i]->get_id()==old_point->get_id()){
            endpoints_[i] = new_point;
            return true;
        }
            
    }
    return false;
}

void LineSegment::AddRectArea(const cv::Rect &area){
    for(int x=area.x; x<area.x+area.width; x++){
        for(int y=area.y; y<area.y+area.height; y++){
            this->AddPoint(cv::Point2f(x,y));
        }
    }
}

cv::Point2f LineSegment::get_start_pt() const { 
    if(endpoints_[0]!=nullptr && endpoints_[0]->type()!=1)
        return endpoints_[0]->get_center();         
    else
        return cv::Point2f(-1, -1);                 // 返回一个图像外的点，以指示该这线段不存在起点
}
cv::Point2f LineSegment::get_end_pt() const { 
    if(endpoints_[1]!=nullptr && endpoints_[1]->type()!=1)
        return endpoints_[1]->get_center();
    else
        return cv::Point2f(-1, -1);           // 如果不存在终点，返回不合法的值
}


/* TODO:current unfinished
 * 
 */
void LineSegment::UpdateLinearity(){
    
}


}  //namespace graph_creator
