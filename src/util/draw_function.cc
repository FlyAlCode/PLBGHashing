#include "draw_function.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

bool DrawMergeOnMap(const cv::Mat &query_img,               
               const cv::Mat &map_img,                            
               const cv::Mat &H_f, 
               int padding_size,                        
               cv::Mat &draw_img){
    cv::Mat H;
    H_f.convertTo(H, CV_64F);
    // calculate four transformed corners
    double corner_data[12] = {0, query_img.cols, 0, query_img.cols, 
                              0, 0, query_img.rows, query_img.rows,
                              1, 1, 1, 1};
    cv::Mat corners(3, 4, CV_64F, corner_data);

    cv::Mat transformed_corners = H * corners;
    for(int i=0; i<4; ++i){
        transformed_corners.at<double>(0,i) /= transformed_corners.at<double>(2,i);
        transformed_corners.at<double>(1,i) /= transformed_corners.at<double>(2,i);
    }
    
    // std::cout<<"H = "<<H<<std::endl;
    // std::cout<<corners<<std::endl;
    // std::cout<<transformed_corners<<std::endl;
    
    int min_x=transformed_corners.at<double>(0,0);
    int max_x=transformed_corners.at<double>(0,0);
    int min_y=transformed_corners.at<double>(1,0);
    int max_y=transformed_corners.at<double>(1,0);
    for(int i=1; i<4; ++i){
        if(transformed_corners.at<double>(0,i)>max_x)
            max_x = transformed_corners.at<double>(0,i);
        if(transformed_corners.at<double>(0,i)<min_x)
            min_x = transformed_corners.at<double>(0,i);
        
        if(transformed_corners.at<double>(1,i)>max_y)
            max_y = transformed_corners.at<double>(1,i);
        if(transformed_corners.at<double>(1,i)<min_y)
            min_y = transformed_corners.at<double>(1,i);
    }
    if(max_x<=min_x || max_y<min_y || max_x<=0 
        || max_y<=0 || min_x>=map_img.cols-1 || min_y>=map_img.rows-1)
        return false;
    
    // draw on map_img
    min_x -= padding_size;
    min_y -= padding_size;
    max_x += padding_size;
    max_y += padding_size;
    
    if(min_x<0)
        min_x=0;
    if(min_y<0)
        min_y=0;
    if(max_x>map_img.cols-1)
        max_x = map_img.cols-1;
    if(max_y>map_img.rows-1)
        max_y=map_img.rows-1;

    cv::Mat roi_img, color_query_img;
    cv::Rect roi(min_x, min_y, max_x-min_x+1, max_y-min_y+1);
    std::cout<<"roi = "<<roi<<std::endl;
    map_img(roi).copyTo(roi_img);
    
    // cv::namedWindow("roi_img");
    // cv::imshow("roi_img", roi_img);
    // cv::waitKey();
        
    if(roi_img.channels()!=3)
        cv::cvtColor(roi_img, roi_img, CV_GRAY2RGB);

    cv::Mat gray_query_img;
    if (query_img.channels() != 3){
        cv::cvtColor(query_img, color_query_img, CV_GRAY2RGB);
        query_img.copyTo(gray_query_img);
    }
    else{
        query_img.copyTo(color_query_img);
        cv::cvtColor(query_img, gray_query_img, CV_RGB2GRAY);
    }

    // cv::imshow("src", gray_query_img);
    // cv::waitKey();

    cv::Vec3b color, color_roi, color_add;
    for (double y = 0; y < query_img.rows; ++y) {
        for(double x=0; x<query_img.cols; ++x){
            if(gray_query_img.at<uchar>(y,x)==0)
                continue;
            double pt_data[3] = {x, y, 1};
            cv::Mat pt(3, 1, CV_64F, pt_data);
            cv::Mat transformed_pt_h = H * pt;
            cv::Point transformed_pt(transformed_pt_h.at<double>(0,0)/transformed_pt_h.at<double>(2,0), 
                                       transformed_pt_h.at<double>(1,0)/transformed_pt_h.at<double>(2,0));
            transformed_pt = transformed_pt - cv::Point(min_x, min_y);
            if(transformed_pt.x<0 || transformed_pt.y<0 
                || transformed_pt.x>roi_img.cols-1 || transformed_pt.y>roi_img.rows-1)
                continue;
            // 图片叠加
            color = color_query_img.at<cv::Vec3b>(y, x);
            color[1] = 0;

            roi_img.at<cv::Vec3b>(transformed_pt) = color;
        }
    }
    roi_img.copyTo(draw_img);
    return true;
}

bool DrawRoadOnSat(const cv::Mat &query_img,               
               const cv::Mat &map_img,                                  // the reference sat image                      
               const cv::Mat &H,                                        // transform pts in query image to map image 
               cv::Mat &draw_img){
    // calculate four transformed corners
    double corner_data[12] = {0, query_img.cols, 0, query_img.cols, 
                              0, 0, query_img.rows, query_img.rows,
                              1, 1, 1, 1};
    cv::Mat corners(3, 4, CV_64F, corner_data);

    cv::Mat transformed_corners = H * corners;
    for(int i=0; i<4; ++i){
        transformed_corners.at<double>(0,i) /= transformed_corners.at<double>(2,i);
        transformed_corners.at<double>(1,i) /= transformed_corners.at<double>(2,i);
    }
    
    // std::cout<<"H = "<<H<<std::endl;
    // std::cout<<corners<<std::endl;
    // std::cout<<transformed_corners<<std::endl;
    
    int min_x=transformed_corners.at<double>(0,0);
    int max_x=transformed_corners.at<double>(0,0);
    int min_y=transformed_corners.at<double>(1,0);
    int max_y=transformed_corners.at<double>(1,0);
    for(int i=1; i<4; ++i){
        if(transformed_corners.at<double>(0,i)>max_x)
            max_x = transformed_corners.at<double>(0,i);
        if(transformed_corners.at<double>(0,i)<min_x)
            min_x = transformed_corners.at<double>(0,i);
        
        if(transformed_corners.at<double>(1,i)>max_y)
            max_y = transformed_corners.at<double>(1,i);
        if(transformed_corners.at<double>(1,i)<min_y)
            min_y = transformed_corners.at<double>(1,i);
    }
    if(max_x<=min_x || max_y<min_y || max_x<=0 
        || max_y<=0 || min_x>=map_img.cols-1 || min_y>=map_img.rows-1)
        return false;
    
    if(min_x<0)
        min_x=0;
    if(min_y<0)
        min_y=0;
    if(max_x>map_img.cols-1)
        max_x = map_img.cols-1;
    if(max_y>map_img.rows-1)
        max_y=map_img.rows-1;

    cv::Mat roi_img, gray_query_img;
    cv::Rect roi(min_x, min_y, max_x-min_x+1, max_y-min_y+1);
    std::cout<<"roi = "<<roi<<std::endl;
    map_img(roi).copyTo(roi_img);
    
    // cv::namedWindow("roi_img");
    // cv::imshow("roi_img", roi_img);
    // cv::waitKey();
        
    if(roi_img.channels()!=3)
        cv::cvtColor(roi_img, roi_img, CV_GRAY2RGB);
    
    if(query_img.channels()==3)
        cv::cvtColor(query_img, gray_query_img, CV_RGB2GRAY);
    else
        query_img.copyTo(gray_query_img);

    cv::Vec3b color, color_roi, color_add;
    cv::Mat H_inv = H.inv();
    for (double y = 0; y < roi_img.rows; ++y)
    {
        for(double x=0; x<roi_img.cols; ++x){
            double pt_data[3] = {x + min_x, y + min_y, 1};
            cv::Mat pt(3, 1, CV_64F, pt_data);
            cv::Mat transformed_pt_h = H_inv * pt;
            cv::Point transformed_pt(transformed_pt_h.at<double>(0,0)/transformed_pt_h.at<double>(2,0), 
                                       transformed_pt_h.at<double>(1,0)/transformed_pt_h.at<double>(2,0));
            if(transformed_pt.x<0 || transformed_pt.y<0 
                || transformed_pt.x>gray_query_img.cols-1 || transformed_pt.y>gray_query_img.rows-1)
                continue;
            // 图片叠加
            if(gray_query_img.at<uchar>(transformed_pt)>30)
                roi_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
        }
    }

    map_img.copyTo(draw_img);
    roi_img.copyTo(draw_img(roi));
    return true;
}

void ShowMatch(const cv::Mat &query_img, const cv::Mat &ref_img,
                const std::vector<cv::Point2f> &query_pts,
                const std::vector<cv::Point2f> &ref_pts,
                cv::Mat &draw_img){
    // 创建画布图片，并将两张图拼接起来
    draw_img = cv::Mat::zeros(std::max(query_img.rows, ref_img.rows), query_img.cols + ref_img.cols, CV_8UC3);
    cv::Mat query_img_tmp, ref_img_tmp;
    if(query_img.type()!=CV_8UC3)
        cv::cvtColor(query_img, query_img_tmp, CV_GRAY2RGB);
    else
        query_img.copyTo(query_img_tmp);

    if(ref_img.type()!=CV_8UC3)
        cv::cvtColor(ref_img, ref_img_tmp, CV_GRAY2RGB);
    else
        ref_img.copyTo(ref_img_tmp);
        
    query_img_tmp.copyTo(draw_img(cv::Rect(0, 0, query_img.cols, query_img.rows)));
    ref_img_tmp.copyTo(draw_img(cv::Rect(query_img.cols, 0, ref_img.cols, ref_img.rows)));

    // 创建若干种颜色
    std::vector<cv::Scalar> colors;
    for (int r = 255; r >= 0; r-=70){
        for (int g = 255; g >= 0; g-=70){
            for (int b = 255; b >= 0; b-=70){
                colors.push_back(cv::Scalar(r, g, b));
            }
        }
    }

    // 画出没对匹配对应的两个端点，及对应的直线
    int k = 0;
    for (int i = 0; i < query_pts.size();i++){
        cv::Point start_pt(query_pts[i].x, query_pts[i].y);
        cv::Point end_pt(ref_pts[i].x + query_img.cols, ref_pts[i].y);
        cv::circle(draw_img, start_pt, 5, colors[k]);
        cv::circle(draw_img, end_pt, 5, colors[k]);
        cv::line(draw_img, start_pt, end_pt, colors[k]);

        ++k;
        if(k>=colors.size())
            k = 0;
    }
}