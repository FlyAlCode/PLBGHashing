#ifndef GHBGL_GEO_LOCATOR_H_
#define GHBGL_GEO_LOCATOR_H_
#include <list>
#include "geometric_hashing_locator.h"


namespace ghbgl {
class GeoLocator{
public:
    void Init(const GeometricHashingParam &geometric_hashing_params, float min_node_distance);

    // 传入参考图，但是未建立geometric hashing
    void AddReferenceTile(const cv::Mat &image,                             // 增加的参考图面片
                          const cv::Point2f &offset);                       // 该tile相对于整个参考图的坐标

    // 当所有的参考图都添加，并提取相应的折线集时，调用该函数，建立对应的geometric hashing
    void InitGeometricHashingLocator();

    // 对给定的图像，在各个尺度上进行搜索
    void Localize(const cv::Mat &query_img,                          
             float scale_min,
             float scale_max,
             float scale_step,
             float min_vaild_vote_rate,
             std::vector<cv::Mat> &similarity_models,       
             std::vector<float> &vote_num_rate);

private:
    // 对给定的图片进行给定尺度的缩放，而后在缩放后的尺度上进行定位
    void LocatorInGivenScale(const cv::Mat &query_img,
                             float scale,
                             float min_vaild_vote_rate,
                             std::list<SimilarityTransformationPtr> &similarity_models,
                             std::list<float> &vote_num_rate);

                             
    std::vector<cv::Mat> ref_tile_imgs_;
    std::vector<PolyLineSet> ref_tile_polyline_sets_;
    std::vector<cv::Point2f> tile_offsets_;
    GeometricHashingLocator geometric_hashing_locator_;

    // params
    float min_node_distance_;
    GeometricHashingParam geometric_hashing_params_;
};

} // namespace ghbgl 

#endif