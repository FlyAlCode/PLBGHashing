#ifndef GHBGL_GEOMETRIC_HASHING_LOCATOR_H_
#define GHBGL_GEOMETRIC_HASHING_LOCATOR_H_
#include <list>
#include "geometric_hashing.h"
#include "basic_class.h"

namespace ghbgl {

class GeometricHashingLocator{
public:
    // 用reference图中，每个tile包含折线，创建相应的geometric hashing
    void Init(const std::vector<PolyLineSet> &tiles_lines,
            const GeometricHashingParam &geometric_hashing_param);            

    void Run(const PolyLineSet &query_lines,
             float min_require_vote_num_rate,
             std::list<EuclideanTransformationPtr> &T_es,
             std::list<float> &vote_num_rate_vaild,
             std::list<int> &tile_indexs) const;

private:
    EuclideanTransformation ComputeTe(const cv::Point2f &src_p1,
                   const cv::Point2f &src_p2,
                   const cv::Point2f &dst_p1,
                   const cv::Point2f &dst_p2) const;

    std::vector<GeometricHashingPtr> ref_ghs_;                      // 参考图被切分为若干tile，每个tile对应一个geometric hashing

    const int MAX_MODEL_NUM = 5;
};

} // namespace ghbgl 

#endif