#ifndef SHP_SHP_H_
#define SHP_SHP_H_
#include <vector>
#include <opencv2/core/core.hpp>

class Shp{
public:
    // 读取shp文件，并提取其边界和所有的折线，并转换到像素坐标系下
    // 如果文件不存在，返回false；否则返回true
    bool Init(const std::string &shp_file_name);

    // 将指定区域的矢量图转换为栅格图
    // area_geo_bound --- 地理坐标系下的转换区域的边界
    // scale --- 地理坐标系到像素坐标系的尺度（每个像素代表多少m）
    // line_width --- 线的宽度
    // result_img --- 转换得到的图像
    // 返回值 --- 整个区域都超出边界，返回false；否则取边界内的区域，并返回true
    bool ConvertAreaToImage(const cv::Rect_<double> &area_geo_bound,
                            double scale,
                            cv::Mat &result_img,
                            int line_width = 5,
                            const cv::Scalar &color = cv::Scalar(255, 255, 255),
                            bool black_background = true) const;

    // 将show_img显示在地图上
    // T --- 图片坐标系到地理坐标系的转换关系
    // padding_size --- 投影图片之外需要添加的背景边界大小
    // color --- 道路图显示的颜色
    // result_show_img --- 显示的结果图
    // black_background --- 背景为黑色还是白色，true为黑色
    // 如果图片在给定转换关系下与地图有重合区域，返回true；否则返回false
    bool ShowOnMap(const cv::Mat &show_img,
                   const cv::Mat &T,
                   int padding_size,
                   const cv::Scalar &color,
                   cv::Mat &result_show_img,
                   bool black_background = true) const;

    // 获取将图片坐标转换为地理坐标的转换矩阵
    cv::Mat GetTransformationFromImageToGeo() const;

    inline const cv::Rect2d &get_geo_bound() const { return geo_bound_; }

    // 图像坐标转换到地理坐标
    cv::Point2d ImageToGeo(const cv::Point &img_pt) const;

    // 地理坐标系到图片坐标系
    cv::Point GeoToImage(const cv::Point2d &geo_pt) const;

private:
    

    cv::Point2d GetTransformedPt(const cv::Point2d &src_pt, const cv::Mat &H) const;

    cv::Rect_<double> geo_bound_;
    cv::Rect pixel_bound_;
    std::vector<std::vector<cv::Point2d>> geo_polylines_;
    std::vector<std::vector<cv::Point>> pixel_polylines_;
};

#endif