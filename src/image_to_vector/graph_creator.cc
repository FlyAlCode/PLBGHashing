#include "graph_creator.h"

#include <opencv2/highgui/highgui.hpp>
#include "keypoint.h"
#include "line_segment.h"


namespace graph_creator{
GraphCreator::GraphCreator(){
    
}

GraphCreator::~GraphCreator(){
    
}

/* @img is supposed to be a binary image, where 1 is for existing, and 0 is for no existing
 * @Currently, we don't check whether img is a binary image; if image doesn't satisfy condition
 * @above, you will get wrong result
 */
void GraphCreator::Init(const int &min_line_length){
//     line_segment_stack_.clear();
    line_segment_set_.clear();
    keypoint_set_.clear();
    
    min_line_length_ = min_line_length;    
}

/*  @This function do as following: 
 *  @ 1. Create a new keypoint
 *  @ 2. The place of the keypoint in the id_map_ is set to the id of the keypoint
 *  @ 3. The keypoint is recored in the graph
 */
void GraphCreator::CreateAllCrossPoints(const std::vector<cv::Rect> &keypoint_area){
    // init id_map_
    cv::Mat id_map_tmp = cv::Mat::zeros(neighbor_img_.size(), CV_32F);   // 0 for background, >0 for keypoint id
    for(int i=0; i<keypoint_area.size(); i++){
        KeyPointPtr kp (new KeyPoint());
        cv::Point pt(keypoint_area[i].x, keypoint_area[i].y);
        int ky_type = neighbor_img_.at<uchar>(pt);
        kp->Init(keypoint_area[i], ky_type);
        
        int id = kp->get_id();
        cv::Mat id_mat = cv::Mat::ones(keypoint_area[i].height, keypoint_area[i].width, CV_32F);
        id_mat = id_mat * id;
        id_mat.copyTo(id_map_tmp(keypoint_area[i]));
        
        keypoint_set_.insert(std::pair<int, KeyPointPtr>(id, kp));
    }
    id_map_tmp.copyTo(id_map_);
}

void GraphCreator::FindAllStartRectsAroundKeypoint(const KeyPointPtr &kp, std::vector<cv::Rect> &start_rects){
    start_rects.clear();
    cv::Rect kp_area = kp->get_area();
    std::vector<cv::Point> around_points;
    std::vector<uchar> around_points_value;
    
    // recored all around points
    cv::Point tmp_pt;
    for(int j=kp_area.x-1; j<=kp_area.x+kp_area.width; j++){            // up
        tmp_pt.x = j;
        tmp_pt.y = kp_area.y-1;
        around_points.push_back(tmp_pt);
        around_points_value.push_back(neighbor_img_.at<uchar>(tmp_pt));
    }
    
    for(int i=kp_area.y; i<=kp_area.y+kp_area.height; i++){
        tmp_pt.x = kp_area.x + kp_area.width;
        tmp_pt.y = i;
        around_points.push_back(tmp_pt);
        around_points_value.push_back(neighbor_img_.at<uchar>(tmp_pt));
    }
    
    for(int j=kp_area.x+kp_area.width-1; j>=kp_area.x-1; j--){
        tmp_pt.x = j;
        tmp_pt.y = kp_area.y + kp_area.height;
        around_points.push_back(tmp_pt);
        around_points_value.push_back(neighbor_img_.at<uchar>(tmp_pt));
    }
    
    for(int i=kp_area.y+kp_area.height-1; i>=kp_area.y-1; i--){         // The up-left point is count twice to loop the circle
        tmp_pt.x = kp_area.x - 1;
        tmp_pt.y = i;
        around_points.push_back(tmp_pt);
        around_points_value.push_back(neighbor_img_.at<uchar>(tmp_pt));
    }
    
    bool mask = false;
    int min_x, min_y, max_x, max_y;
    for(int i=0; i<around_points.size(); i++){
        if(mask){
            if(around_points_value[i]==2){
                if(around_points[i].x>max_x)
                    max_x = around_points[i].x;
                if(around_points[i].x<min_x)
                    min_x = around_points[i].x;
                if(around_points[i].y>max_y)
                    max_y = around_points[i].y;
                if(around_points[i].y<min_y)
                    min_y = around_points[i].y;
            }
            else{
                start_rects.push_back(cv::Rect(min_x, min_y, max_x-min_x+1, max_y-min_y+1));
                mask = false;
            }            
        }
        else{
            if(around_points_value[i]==2){
                min_x = around_points[i].x;
                max_x = around_points[i].x;
                min_y = around_points[i].y;
                max_y = around_points[i].y;
                mask = true;
            }
        }
    }  
    
}
/*  @Because no area like this exists, the problem can be divided into 4 smaller problem
 *  @  eg.  2 2 0 0 2 2
 *  @       2 3 3 3 3 0
 *  @       0 0 2 0 0 0
 *  @ example above is not likely to be found, because usually the left-up point has more than two neighbor
 *  @ There exist some bugs in this function: standalone corner point will be counted more than once
 */
/*
void GraphCreator::FindAllStartRectsAroundKeypoint(const KeyPointPtr &kp, std::vector<cv::Rect> &start_rects){
    start_rects.clear();
    cv::Rect kp_area = kp->get_area();
    bool mask = false;
    // deal with the up line
    int min_i, max_i, min_j, max_j;
    for(int j=kp_area.x-1; j<=kp_area.x+kp_area.width; j++){                    
        if(mask){
            if(neighbor_img_.at<uchar>(kp_area.y-1, j)==2){
                max_j = j;
            }
            else{
                start_rects.push_back(cv::Rect(min_j, min_i, max_j-min_j+1, max_i-min_i+1 ));
                mask = false;
            }
        }
        else{
            if(neighbor_img_.at<uchar>(kp_area.y-1, j)==2){
                min_i = kp_area.y-1;
                max_i = kp_area.y-1;
                min_j = j;
                max_j = j;
                mask = true;
            }
        }
        
    }
    
    // deal with the right line
    for(int i=kp_area.y-1; i<=kp_area.y+kp_area.height; i++){
        if(mask){
            if(neighbor_img_.at<uchar>(i, kp_area.x+kp_area.width)==2){
                max_i = i;
            }
            else{
                start_rects.push_back(cv::Rect(min_j, min_i, max_j-min_j+1, max_i-min_i+1 ));
                mask = false;
            }
        }
        else{
            if(neighbor_img_.at<uchar>(i, kp_area.x+kp_area.width)==2){
                min_i = i;
                max_i = i;
                min_j = kp_area.x+kp_area.width;
                max_j = kp_area.x+kp_area.width;
                mask = true;
            }
        }
    }
    
    // deal with the down line
    for(int j=kp_area.x-1; j<=kp_area.x+kp_area.width; j++){                    
        if(mask){
            if(neighbor_img_.at<uchar>(kp_area.y+kp_area.height, j)==2){
                max_j = j;
            }
            else{
                start_rects.push_back(cv::Rect(min_j, min_i, max_j-min_j+1, max_i-min_i+1 ));
                mask = false;
            }
        }
        else{
            if(neighbor_img_.at<uchar>(kp_area.y+kp_area.height, j)==2){
                min_i = kp_area.y+kp_area.height;
                max_i = kp_area.y+kp_area.height;
                min_j = j;
                max_j = j;
                mask = true;
            }
        }
        
    }
    
    // deal with the left line
    for(int i=kp_area.y-1; i<=kp_area.y+kp_area.height; i++){
        if(mask){
            if(neighbor_img_.at<uchar>(i, kp_area.x-1)==2){
                max_i = i;
            }
            else{
                start_rects.push_back(cv::Rect(min_j, min_i, max_j-min_j+1, max_i-min_i+1 ));
                mask = false;
            }
        }
        else{
            if(neighbor_img_.at<uchar>(i, kp_area.x-1)==2){
                min_i = i;
                max_i = i;
                min_j = kp_area.x-1;
                max_j = kp_area.x-1;
                mask = true;
            }
        }
    }
} */

/*  @Here, the visited_map_(initialized in the Run()) will be set
 *  @We assuse centre has been marked as visited
 *  @Return false when not any more point can be expanded to 
 *  @To solve the error in keypoint, the expanded rect can't 
 *  @coincide with the last_centre
 *  @ return value: -1 --- cross point found 0--- no expand points found and no cross point found 1 --- found expand points
 */
int GraphCreator::ExpandToNextPoint(const cv::Rect &centre, const cv::Rect &last_centre, cv::Rect &next_rect){
    int min_i;
    int max_i;
    int min_j;
    int max_j;
    bool rect_inited = false;
    cv::Rect last_centre_around = cv::Rect(last_centre.x-1, last_centre.y-1, last_centre.width+2, last_centre.height+2);
    for(int i=centre.y-1; i<=centre.y+centre.height; i++){
        for(int j=centre.x-1; j<=centre.x+centre.width; j++){
            bool last_centre_around_contain = last_centre_around.contains(cv::Point(j,i));
            if(!last_centre_around_contain && ((neighbor_img_.at<uchar>(i,j)>2) || neighbor_img_.at<uchar>(i,j)==1)){            // cross or terminal point found
                return -1;
            }
            if(!last_centre_around_contain && visited_map_.at<uchar>(i,j)==0 && neighbor_img_.at<uchar>(i,j)==2 ){
                if(rect_inited){
                    if(i < min_i)
                        min_i = i;
                    if(i > max_i)
                        max_i = i;
                    if(j < min_j)
                        min_j = j;
                    if(j > max_j)
                        max_j = j;
                }
                else{
                    min_i = i;
                    max_i = i;
                    min_j = j;
                    max_j = j;
                    rect_inited = true;
                }
//                 visited_map_.at<uchar>(i,j) = 1; 
            }
            
        }
    }
    
//     std::cout<<"next_rect = "<<next_rect<<"       rect_inited: "<<rect_inited<<std::endl;
    if(!rect_inited){                       // The rect is not initialized, which indicate no rect can be expanded to. This is occoured when encounting the boundary of the image 
//         std::cout<<"Error occoured!!!"<<std::endl;
//         std::cout<<"centre = "<<centre<<std::endl;
        return 0;
    }          
    else{
        // set the new rect as visited
        for(int i=min_i; i<=max_i; i++){
            for(int j=min_j; j<=max_j; j++){
                visited_map_.at<uchar>(i, j) = 1;
            }
        }
        next_rect.x = min_j;
        next_rect.y = min_i;
        next_rect.width = max_j - min_j + 1;
        next_rect.height = max_i- min_i + 1;
        return 1;
    }
        
}

/*  @0. Check if the line has been created by checking whether the start_rect has been visited
 *  @1. Create a new line object
 *  @2. Expand to next area until no point can be expanded to
 *  @3. Search for keypoint around the place where the expansion ends
 *  @4. Add a reference of keypoint to the line 
 */
bool GraphCreator::ExpandOneLineSegment(const KeyPointPtr &start_point, const cv::Rect &start_rect, LineSegmentPtr &line){
    // 0. Check whether the line has been created; if not, set it as visited
    for(int x=start_rect.x; x<start_rect.x+start_rect.width; x++){
        for(int y=start_rect.y; y<start_rect.y+start_rect.height; y++){
            if(visited_map_.at<uchar>(y, x)==1)
                return false;
        }
    }
    
    for(int x=start_rect.x; x<start_rect.x+start_rect.width; x++){
        for(int y=start_rect.y; y<start_rect.y+start_rect.height; y++){
            visited_map_.at<uchar>(y, x) = 1;
        }
    }
    
    // 1.create new line
    LineSegmentPtr line_tmp(new LineSegment());
    line_tmp->Init(start_point);
    line_tmp->AddRectArea(start_rect);
    
    // 2. expand
    cv::Rect last_centre = start_point->get_area();
    cv::Rect current_rect = start_rect;
    cv::Rect next_rect;
    int expand_line_flag;
//     bool finish_add_end_keypoit = false;
    while(true){
        expand_line_flag = ExpandToNextPoint(current_rect, last_centre, next_rect);
        if(expand_line_flag ==1){
            last_centre = current_rect;
            current_rect = next_rect;        
            //         std::cout<<"next_rect = "<<next_rect<<std::endl;
            line_tmp->AddRectArea(next_rect);
        }
        
        if(expand_line_flag==-1){
            // 3.search area around current_rect to find a keypoint id
            for(int i=current_rect.y-1; i<=current_rect.y+current_rect.height; i++){ 
                for(int j=current_rect.x-1; j<=current_rect.x+current_rect.width; j++){
                    if(id_map_.at<float>(i,j)>0){
                        int id = id_map_.at<float>(i,j);  
                        // 4.add reference
                        line_tmp->SetEndPoint(keypoint_set_[id]);               
                        line = line_tmp;
                        return true;
                    }
                }
            }
        }
    
        if(expand_line_flag==0){
            line_tmp->SetEndPoint(nullptr);
            line = line_tmp;
            return true;
        }
    }
    
}

/* @visited every keypoint, check whether its joint area has been visited;
 * @if not, start form than joint area to expand into a new line;
 */
void GraphCreator::ExpandAllLineSegments(){
    std::map<int, KeyPointPtr>::iterator iter;
    for(iter=keypoint_set_.begin(); iter!=keypoint_set_.end(); iter++){
        std::vector<cv::Rect> start_rects;
        FindAllStartRectsAroundKeypoint(iter->second, start_rects);
        LineSegmentPtr line_tmp;
        for(int i=0; i<start_rects.size(); i++){
            if(visited_map_.at<uchar>(start_rects[i].y, start_rects[i].x)==0){   // unvisited
                if(ExpandOneLineSegment(iter->second, start_rects[i], line_tmp))
                    line_segment_set_.insert(std::pair<int, LineSegmentPtr>(line_tmp->get_id(), line_tmp));
            }               
        }
    }
    
}

/*  @1. Initialize, visited_map_, neighbor_img_
 *  @2. Call Init() to clear the graph(points and lines)
 *  @3. Find all keypoints 
 *  @4. Find all line segments 
 *  @5. Copy result to line_segment_set and keypoint_set
 */
void GraphCreator::Run(const cv::Mat &neighbor_img, 
                       const std::vector<cv::Rect> &keypoint_area, 
                       const int &min_line_length,
                       std::map<int, LineSegmentPtr> &line_segment_set,
                       std::map<int, KeyPointPtr> &keypoint_set){
    this->Init(min_line_length);
    
    neighbor_img.copyTo(neighbor_img_);
    cv::Mat tmp = cv::Mat::zeros(neighbor_img.size(), neighbor_img.type());
    tmp.copyTo(visited_map_);  
    
    this->CreateAllCrossPoints(keypoint_area);
    
    this->ExpandAllLineSegments();
    
    // std::cout<<"line_segment_set_: size = "<<line_segment_set_.size()<<std::endl;
    line_segment_set = line_segment_set_;
    keypoint_set = keypoint_set_;
}

/***************************************************************
 *                       Used just for debug                   *
 * *************************************************************/
void GraphCreator::DrowKeypointAroundPoints(const cv::Mat &neighbor_img,
                                            const std::vector<cv::Rect> &keypoint_area){
    cv::Mat draw_img = cv::Mat::zeros(neighbor_img.size(), CV_8UC3);    
    this->Init(10);
    
    neighbor_img.copyTo(neighbor_img_);
    // draw keypoint area
    for(int i=0; i<keypoint_area.size(); i++){
        DrawRect(keypoint_area[i], cv::Vec3b(255,0,0), draw_img);
    }
    cv::namedWindow("draw_img");
    cv::imshow("draw_img", draw_img);
    cv::waitKey();
      
    this->CreateAllCrossPoints(keypoint_area);
    
    // draw point around keypoint
    std::map<int, KeyPointPtr>::iterator iter;
    for(iter=keypoint_set_.begin(); iter!=keypoint_set_.end(); iter++){
        std::vector<cv::Rect> start_rects;
        FindAllStartRectsAroundKeypoint(iter->second, start_rects);
        for(int i=0; i<start_rects.size(); i++){                        // draw the rects 
            DrawRect(start_rects[i], cv::Vec3b(0,0,255), draw_img);                           
        }
    }
    
    cv::namedWindow("keypoint and surrounding points");
    cv::imshow("keypoint and surrounding points", draw_img);
    cv::waitKey();
}

/*  @No size and type check done for draw_img, which is suppose to be CV_8UC3
 */
void GraphCreator::DrawRect(const cv::Rect& rect, const cv::Vec3b &color, cv::Mat& draw_img){
    for(int x=rect.x; x<rect.x+rect.width; x++){
        for(int y=rect.y; y<rect.y+rect.height; y++){
            draw_img.at<cv::Vec3b>(y, x) = color;
        }
    }
}

/*  @Used for observing the growing of line segment 
 */
// void GraphCreator::DrawLineGrowing(const cv::Mat &neighbor_img, 
//                                    const std::vector<cv::Rect> &keypoint_area){
//     
// }




}  //namespace graph_creator
