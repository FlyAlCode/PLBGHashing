#include "geo_locator.h"
#include "image_to_vector.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <list>

// debug
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

namespace ghbgl {
    
void GeoLocator::Init(const GeometricHashingParam &geometric_hashing_params,
                        float min_node_distance){
    geometric_hashing_params_ = geometric_hashing_params;
    min_node_distance_ = min_node_distance;
    ref_tile_imgs_.clear();
    ref_tile_polyline_sets_.clear();
    tile_offsets_.clear();
}

void GeoLocator::AddReferenceTile(const cv::Mat &image,                             
                        const cv::Point2f &offset){
    // 提取折线集合
    ref_tile_polyline_sets_.push_back(PolyLineSet());
    graph_creator::ImageToVector img_to_vector;
    img_to_vector.Run(image, 5, min_node_distance_, ref_tile_polyline_sets_[ref_tile_polyline_sets_.size() - 1]);

    // 存储每个tile的偏移
    tile_offsets_.push_back(offset);
}

void GeoLocator::InitGeometricHashingLocator(){
    geometric_hashing_locator_.Init(ref_tile_polyline_sets_, geometric_hashing_params_);
}

void GeoLocator::Localize(const cv::Mat &query_img,                          
             float scale_min,
             float scale_max,
             float scale_step,
             float min_vaild_vote_rate,
             std::vector<cv::Mat> &similarity_models,       
             std::vector<float> &vote_num_rate){
    std::list<float> vote_rate_list;              // 因为要对模型通过票数进行排序，这里我们采用插值排序法，使用list避免了频繁插值带来的元素的拷贝
    std::list<cv::Mat> Ts_list;

    std::list<SimilarityTransformationPtr> current_scale_similarity_models;
    std::list<float> current_scale_vote_rate;
    for (float s = scale_max; s >= scale_min; s -= scale_step) {
        LocatorInGivenScale(query_img, s, min_vaild_vote_rate, current_scale_similarity_models, current_scale_vote_rate);
        
        if(Ts_list.empty()) {                   // 第一次计算，不需要进行排序
            std::list<float>::iterator vote_rate_list_iter = current_scale_vote_rate.begin();
            std::list<SimilarityTransformationPtr>::iterator model_iter = current_scale_similarity_models.begin();
            for (; vote_rate_list_iter != current_scale_vote_rate.end(); ++vote_rate_list_iter){
                vote_rate_list.push_back(*vote_rate_list_iter);
                Ts_list.push_back((*model_iter)->ToCvMat());
                ++model_iter;
            }
        }
        else{                                   // 插值排序法
            std::list<float>::iterator vote_rate_iter = vote_rate_list.begin();
            std::list<cv::Mat>::iterator Ts_iter = Ts_list.begin();
            std::list<SimilarityTransformationPtr>::iterator current_model_iter = current_scale_similarity_models.begin();
            for (std::list<float>::iterator current_vote_rate_list_iter = current_scale_vote_rate.begin(); current_vote_rate_list_iter != current_scale_vote_rate.end();)
            {
                if(vote_rate_iter==vote_rate_list.end()){                                    // 如果到达待插入对象的末尾，则直接插入
                    vote_rate_list.insert(vote_rate_iter, *current_vote_rate_list_iter);
                    Ts_list.insert(Ts_iter, (*current_model_iter)->ToCvMat());
                    ++current_model_iter;
                    ++current_vote_rate_list_iter;
                }
                else{
                    if (*current_vote_rate_list_iter > *vote_rate_iter) {
                        vote_rate_list.insert(vote_rate_iter, *current_vote_rate_list_iter);
                        Ts_list.insert(Ts_iter, (*current_model_iter)->ToCvMat());
                        ++current_model_iter;
                        ++current_vote_rate_list_iter;
                    }
                    else{
                        ++vote_rate_iter;
                        ++Ts_iter;
                    }
                }
                
            }
        }                  
    }

    similarity_models.resize(Ts_list.size());
    vote_num_rate.resize(vote_rate_list.size());
    int i = 0;
    std::list<float>::iterator iter_vote_num = vote_rate_list.begin();
    std::list<cv::Mat>::iterator iter_model = Ts_list.begin();
    for (int i = 0; i < similarity_models.size(); i++){
        iter_model->copyTo(similarity_models[i]);
        vote_num_rate[i] = *iter_vote_num;
        ++iter_vote_num;
        ++iter_model;
    }
}

/* private */

void GeoLocator::LocatorInGivenScale(const cv::Mat &query_img,
            float scale,
            float min_vaild_vote_rate,
            std::list<SimilarityTransformationPtr> &similarity_models,
            std::list<float> &vote_num_rate){
    // do some clear
    similarity_models.clear();
    vote_num_rate.clear();

    // query image to vector
    cv::Mat scaled_query_img;
    cv::resize(query_img, scaled_query_img, cv::Size(scale*query_img.cols, scale*query_img.rows), scale, scale);
    PolyLineSet query_polyline_set;
    graph_creator::ImageToVector img_to_vector;
    img_to_vector.Run(scaled_query_img, 5, min_node_distance_, query_polyline_set);


    // query in geometrc hashing
    std::list<ghbgl::EuclideanTransformationPtr> T_es;
    std::list<int> tile_indexs;
    geometric_hashing_locator_.Run(query_polyline_set, min_vaild_vote_rate, T_es, vote_num_rate, tile_indexs);

    std::list<int>::iterator tile_index_iter = tile_indexs.begin();
    for (std::list<EuclideanTransformationPtr>::iterator T_es_iter = T_es.begin(); T_es_iter != T_es.end(); ++T_es_iter)  {
        SimilarityTransformationPtr current_Ts(new SimilarityTransformation);
        current_Ts->angle_ = (*T_es_iter)->angle_;
        current_Ts->scale_ = 1 / scale;
        current_Ts->x_ = (*T_es_iter)->x_ + tile_offsets_[*tile_index_iter].x;
        current_Ts->y_ = (*T_es_iter)->y_ + tile_offsets_[*tile_index_iter].y;

        similarity_models.push_back(current_Ts);

        ++tile_index_iter;
    }

}

} // namespace ghbgl 