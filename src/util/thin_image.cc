#include "thin_image.h"
#include <opencv2/imgproc/imgproc.hpp>


void FillImageHole(const cv::Mat &input_img, cv::Mat &output_img){
    input_img.copyTo(output_img);
    
    for(int i=1; i<input_img.rows-1;i++){
        for(int j=1; j<input_img.cols-1; j++){
            if(input_img.at<char>(i,j)==0){
                uchar up = input_img.at<uchar>(i-1,j);
                uchar down = input_img.at<uchar>(i+1,j);
                /*if(up==1 && down==1){
                    output_img.at<uchar>(i,j)=1;
                }
                else{
                    uchar left = input_img.at<uchar>(i,j-1);
                    uchar right = input_img.at<uchar>(i,j+1);
                    if(left==1 && right==1)
                        output_img.at<uchar>(i,j) = 1;
                } */      
                uchar left = input_img.at<uchar>(i,j-1);
                uchar right = input_img.at<uchar>(i,j+1);
                if(up==1 && down==1 && right==1 && left==1)
                    output_img.at<uchar>(i,j) = 1;
                
            }
                      
        }
    }
}

void ThinImage(const cv::Mat & src,
               cv::Mat &thined_img,
               int binary_threshold,
               const int maxIterations ){
    cv::Mat dst;
    src.copyTo(dst);
    if(src.channels()==3)
        cv::cvtColor(src, dst, CV_RGB2GRAY);
    else
        src.copyTo(dst);

    cv::threshold(dst, dst, binary_threshold, 1, CV_THRESH_BINARY);

    int width = src.cols;
    int height = src.rows;
    
    int count = 0;  
    while (true)
    {
        count++;
        if (maxIterations != -1 && count > maxIterations)  
            break;
        std::vector<uchar *> mFlag; 
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            { 
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6) {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0){
                        mFlag.push_back(p + j);
                    }
                }
            }
        }

        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i) {
            **i = 0;
        }

        if (mFlag.empty()) {
            break;
        }
        else  {
            mFlag.clear();//œ«mFlagÇå¿Õ  
        }
 
        for (int i = 0; i < height; ++i) {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j) { 
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0) {
                        mFlag.push_back(p + j);
                    }
                }
            }
        }

        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }

        if (mFlag.empty())  {
            break;
        }
        else  {
            mFlag.clear();//œ«mFlagÇå¿Õ  
        }
    }
    dst.copyTo(thined_img);
}