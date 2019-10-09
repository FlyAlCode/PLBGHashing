#ifndef GHBGL_GEOMETRIC_HASHING_H_
#define GHBGL_GEOMETRIC_HASHING_H_
#include <opencv2/core/core.hpp>
#include <memory>
#include <vector>
#include "basic_class.h"


namespace ghbgl {

/* geometric hashing 类 */
class GeometricHashing{
public:

    void Init(float grid_size, float width, float height);
    void Init(const GeometricHashingParam &param);

    // 给定参考折线集，创建geometric hashing
    void Create(const std::vector<std::vector<cv::Point2f>> &polylines,
                const std::vector<float> start_pt_linearity,
                const std::vector<float> end_pt_linearity,
                int tile_index);

    // 给定基底，和折线集，计算所有点的投票结果
    void Vote(const Bias &bais,
              const std::vector<std::vector<cv::Point2f> > &polylines,
              int min_vote_num_required,
              BiasPtrSet &bias_set,                                         // 存储票数不为0的bias，按票数高低排序
              std::vector<int> &vote_num,
              std::vector<int> &tile_indexs) const;                                   // 存储上述bias对应的票数

private:
    /* functions */
    // 获取某一位置下所有的bais
    inline int get_grid_index(const cv::Point2f &pt) const {
        int x_index = std::ceil((pt.x + w_ / 2) / grid_size_);
        int y_index = std::ceil((pt.y + h_ / 2) / grid_size_);

        if (y_index < y_grid_num_ && y_index >= 0 && x_index < x_grid_num_ && x_index >= 0)
            return y_index * x_grid_num_ + x_index;
        else
            return -1;
    }

    /* data */
    std::vector<std::vector<int> > bais_ids_;                           // 存储每个网格下所有的bias的索引
    BiasPtrSet bais_sets_;                                              // 存储所有的bais
    std::vector<int> tile_indexs_;                                      // 每个bais属于那个tile

    // hashing parameters
    float grid_size_;
    float w_;
    float h_;
    int x_grid_num_;
    int y_grid_num_;
};

typedef std::shared_ptr<GeometricHashing> GeometricHashingPtr;

} // namespace ghbgl 


#endif