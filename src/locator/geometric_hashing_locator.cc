#include "geometric_hashing_locator.h"
#include <cstdlib>
#include <chrono>

// debug
#include <iostream>

namespace ghbgl {
void GeometricHashingLocator::Init(const std::vector<PolyLineSet> &tiles_lines, const GeometricHashingParam &geometric_hashing_param){
    ref_ghs_.clear();
    GeometricHashingPtr current_hash(new GeometricHashing);
    current_hash->Init(geometric_hashing_param);
    for (int i = 0; i < tiles_lines.size(); i++)   {
        current_hash->Create(tiles_lines[i].lines_, tiles_lines[i].start_pts_linearity_, tiles_lines[i].end_pts_linearity_,i);
    }
    ref_ghs_.push_back(current_hash);
}          

void GeometricHashingLocator::Run(const PolyLineSet &query_lines,
                  float min_require_vote_num_rate,
                  std::list<EuclideanTransformationPtr > &T_es,
                  std::list<float> &vote_num_rate_vaild,
                  std::list<int> &tile_indexs)const{
    // clear
    T_es.clear();
    vote_num_rate_vaild.clear();
    tile_indexs.clear();

    // 统计总共的票数
    float total_vote_num = 0;
    for (int i = 0; i < query_lines.lines_.size(); i++)  {
        total_vote_num += query_lines.lines_[i].size();
    }
    float min_require_vote_num = total_vote_num * min_require_vote_num_rate;

    std::vector<int> polyline_index;
    for (int i = 0; i < query_lines.lines_.size(); i++)
        polyline_index.push_back(i);

    // 乱序
    unsigned seed = std::chrono::system_clock::now ().time_since_epoch ().count ();
    std::shuffle(polyline_index.begin(), polyline_index.end(),std::default_random_engine (seed));

    // 选择每一个线性度满足要求的基底，进行投票，记录票数大于一定阈值的所有bias，并计算对应的模型，最后对所有模型通过其票数进行排序
    const float linearity_threshold = 2.0;
    EuclideanTransformation T_e;
    // int max_try = 0;
    BiasPtrSet bias_set;
    std::vector<int> vote_num;
    std::vector<int> tile_indexs_vote;
    BiasPtr current_query_bias;
    bool enough_models_found = false;
    int current_index;
    for (int i = 0; i < polyline_index.size(); i++) {               
        current_index = polyline_index[i];

        if (query_lines.lines_[current_index].size() < 2){              // 当前直线点少于2，无法创建一个bias
            continue;
        }       

        // 对每条随机选择的折线，分别选取两个端点作为基底，并进行投票
        for (int m = 0; m < 2; m++){
            // 生成基底
            if(m==0){
                if (query_lines.start_pts_linearity_[current_index] < 0 || query_lines.start_pts_linearity_[current_index] > linearity_threshold){
                    continue;
                }
                else{
                    current_query_bias.reset(new Bias(query_lines.lines_[current_index][0], query_lines.lines_[current_index][1]));
                }
            }

            if(m==1){
                if (query_lines.end_pts_linearity_[current_index] < 0 || query_lines.end_pts_linearity_[current_index] > linearity_threshold){
                    continue;
                }
                else{
                    int N = query_lines.lines_[current_index].size();
                    current_query_bias.reset(new Bias(query_lines.lines_[current_index][N-1], query_lines.lines_[current_index][N-2]));
                }
            }

            // 每个geometric hashing进行投票，并将符合要求的基底按票数保存下来
            for (int j = 0; j < ref_ghs_.size(); j++) {
                ref_ghs_[j]->Vote(*current_query_bias, query_lines.lines_, min_require_vote_num, bias_set, vote_num, tile_indexs_vote);  
                
                if(T_es.empty()) {                          // 如果是第一次插入值
                    for (int k = 0; k < vote_num.size(); k++){
                        if(vote_num[k]>min_require_vote_num){
                            vote_num_rate_vaild.push_back(vote_num[k] / total_vote_num);
                            tile_indexs.push_back(tile_indexs_vote[k]);

                            T_e = ComputeTe(current_query_bias->p_start_, current_query_bias->p_end_,
                                        bias_set[k]->p_start_, bias_set[k]->p_end_);
                            EuclideanTransformationPtr T_e_ptr(new EuclideanTransformation(T_e));
                            T_es.push_back(T_e_ptr);
                        }
                    }
                    continue;
                }

                // 遍历所有的bias_set,对所有得票数大于一定值的bias，计算对应的模型
                // 通过插入排序法，将新的符合条件的模型记录下来
                std::list<int>::iterator tile_index_iter = tile_indexs.begin();
                std::list<float>::iterator vote_rate_iter = vote_num_rate_vaild.begin();
                std::list<EuclideanTransformationPtr>::iterator model_iter = T_es.begin();
                for (int k = 0; k < vote_num.size(); )  {
                    if(vote_num[k]<min_require_vote_num){
                        ++k;
                        continue;
                    }
                    
                    // 计算当前模型
                    T_e = ComputeTe(current_query_bias->p_start_, current_query_bias->p_end_,
                                        bias_set[k]->p_start_, bias_set[k]->p_end_);
                    EuclideanTransformationPtr T_e_ptr(new EuclideanTransformation(T_e));

                    if(model_iter==T_es.end()){                     // 到达插入list的末尾，直接将所有按顺序插入
                        tile_indexs.insert(tile_index_iter, tile_indexs_vote[k]);
                        vote_num_rate_vaild.insert(vote_rate_iter, vote_num[k] / total_vote_num);
                        T_es.insert(model_iter, T_e_ptr);
                        ++k;
                    }
                    else{
                        float current_vote_rate = vote_num[k] / total_vote_num;
                        if(current_vote_rate>*vote_rate_iter){
                            tile_indexs.insert(tile_index_iter, tile_indexs_vote[k]);
                            vote_num_rate_vaild.insert(vote_rate_iter, vote_num[k] / total_vote_num);
                            T_es.insert(model_iter, T_e_ptr);
                            ++k;
                        }
                        else{
                            ++tile_index_iter;
                            ++vote_rate_iter;
                            ++model_iter;
                        }
                    }
                }  

                if(vote_num_rate_vaild.size()>MAX_MODEL_NUM){
                    enough_models_found = true;
                    break;
                }
            }
            if(enough_models_found)
                break;
        }

        if(enough_models_found)
            break;
    }
}

/****************************** private *****************************************/  
EuclideanTransformation GeometricHashingLocator::ComputeTe(const cv::Point2f &src_p1,
                   const cv::Point2f &src_p2,
                   const cv::Point2f &dst_p1,
                   const cv::Point2f &dst_p2)  const {
    cv::Point2f src_dp = src_p1 - src_p2;
    cv::Point2f dst_dp = dst_p1 - dst_p2;
    float dst_angle = atan2(dst_dp.y, dst_dp.x);
    float src_angle = atan2(src_dp.y, src_dp.x);
    float angle = dst_angle - src_angle;
    float R_data[4] = {cosf(angle), -sinf(angle), sinf(angle), cosf(angle)};

    float t_data[2];
    // 由于在计算基底时，只用了p1，因此这里计算t不能使用p2，因为p2不一定是对应的
    t_data[0] = dst_p1.x - (R_data[0] * src_p1.x + R_data[1] * src_p1.y);
    t_data[1] = dst_p1.y - (R_data[2] * src_p1.x + R_data[3] * src_p1.y);

    EuclideanTransformation result;
    result.angle_ = angle;
    result.x_ = t_data[0];
    result.y_ = t_data[1];
    return result;
}
} // namespace ghbgl 



