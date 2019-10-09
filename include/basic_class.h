#ifndef GHBGL_BASIC_H_
#define GHBGL_BASIC_H_
#include <opencv2/core/core.hpp>
#include <vector>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>

namespace ghbgl {
    
/*  定义基底类，主要实现基底的构造，以及将给定的点转换到该基底下
*/
struct Bias{
    cv::Point2f p_start_;                               // 起始点
    cv::Point2f p_end_;                                 // 终点
    cv::Point2f px_;                                    // x方向单位向量
    cv::Point2f py_;                                    // y方向单位向量

    // static int id_;

    Bias(const cv::Point2f &p1, const cv::Point2f &p2) {
        p_start_ = p1;
        p_end_ = p2;
        px_ = p_end_ - p_start_;
        px_ = px_ * (1 / cv::norm(px_));
        py_.x = -px_.y;
        py_.y = px_.x;
    }

    inline cv::Point2f get_coordinate(const cv::Point2f &pt) const{
        cv::Point2f tmp = pt - p_start_;
        return cv::Point2f(tmp.dot(px_), tmp.dot(py_));
    }

    void Show(cv::Mat &img) const{
        if(img.channels()!=3)
            cv::cvtColor(img, img, CV_GRAY2RGB);
        cv::Point2f end = 30 * px_ + p_start_;
        cv::line(img, p_start_, end, cv::Scalar(255, 0, 0));
        cv::circle(img, end, 3, cv::Scalar(0, 0, 255));
    }
};

typedef std::shared_ptr<Bias> BiasPtr;
typedef std::vector<BiasPtr> BiasPtrSet;
typedef std::shared_ptr<BiasPtrSet> BiasPtrSetPtr;

/* 表示欧式变换的类 */
struct EuclideanTransformation{
    float angle_;
    float x_;
    float y_;

    void ToCvMat(cv::Mat &mat){
        cv::Mat Te(3, 3, CV_32F);
        Te.at<float>(0, 0) = cos(angle_);
        Te.at<float>(0, 1) = -sin(angle_);
        Te.at<float>(1, 0) = sin(angle_);
        Te.at<float>(1, 1) = cos(angle_);

        Te.at<float>(0, 2) = x_;
        Te.at<float>(1, 2) = y_;

        Te.at<float>(2, 0) = 0;
        Te.at<float>(2, 1) = 0;
        Te.at<float>(2, 2) = 0;

        Te.copyTo(mat);
    }
};
typedef std::shared_ptr<EuclideanTransformation> EuclideanTransformationPtr;

/* 相似变换 */
struct SimilarityTransformation{
    float angle_;
    float x_;
    float y_;
    float scale_;

    cv::Mat ToCvMat(){
        cv::Mat Te(3, 3, CV_32F);
        Te.at<float>(0, 0) = cos(angle_) / scale_;
        Te.at<float>(0, 1) = -sin(angle_) / scale_;
        Te.at<float>(1, 0) = sin(angle_) / scale_;
        Te.at<float>(1, 1) = cos(angle_) / scale_;

        Te.at<float>(0, 2) = x_;
        Te.at<float>(1, 2) = y_;

        Te.at<float>(2, 0) = 0;
        Te.at<float>(2, 1) = 0;
        Te.at<float>(2, 2) = 1;

        return Te;
    }
};
typedef std::shared_ptr<SimilarityTransformation> SimilarityTransformationPtr;

/* 表示折线集合的类 */
struct PolyLineSet{
    std::vector<std::vector<cv::Point2f> > lines_;
    std::vector<float> start_pts_linearity_;                                      // 初始点附近直线的线性度
    std::vector<float> end_pts_linearity_;                                        // 终点附近直线的线性度
};

/* 表示折线的类 */
struct PolyLine{
    std::vector<cv::Point2f> pts_;                              // 折线上的点集
    float startlinearity_;                                      // 初始点附近直线的线性度
    float end_linearity_;                                        // 终点附近直线的线性度
};

/* geometric hashing的基本参数 */
struct GeometricHashingParam{
    float grid_size_;
    float width_;
    float height_;
};

} // namespace ghbgl 


#endif