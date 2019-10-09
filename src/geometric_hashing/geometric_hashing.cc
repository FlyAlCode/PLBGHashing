#include "geometric_hashing.h"

// debug
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace ghbgl {

void GeometricHashing::Init(float grid_size, float width, float height){
    w_ = width;
    h_ = height;
    grid_size_ = grid_size;

    x_grid_num_ = std::ceil(w_ / grid_size_);
    y_grid_num_ = std::ceil(h_ / grid_size_);

    // 分配内存
    bais_ids_.resize(x_grid_num_ * y_grid_num_);
    bais_sets_.clear();
}

void GeometricHashing::Init(const GeometricHashingParam &param){
    Init(param.grid_size_, param.width_, param.height_);
}

void GeometricHashing::Create(const std::vector<std::vector<cv::Point2f> > &polylines ,
                const std::vector<float> start_pt_linearity,
                const std::vector<float> end_pt_linearity,
                int tile_index){
    for (int i = 0; i < polylines.size(); i++){
        int num = polylines[i].size();
        if (num< 2)                         // 如果一条折线的node少于2个，则无法创建基底
            continue;
        // 创建第一个基底
        cv::Point2f projected_pt;
        if (start_pt_linearity[i] >= 0 && start_pt_linearity[i] <= 2) {
            BiasPtr current_bais_1(new Bias(polylines[i][0], polylines[i][1]));
            bais_sets_.push_back(current_bais_1);
            tile_indexs_.push_back(tile_index);
            // 投影其他点到该基底坐标系下
            for (int j = 0; j < polylines.size(); j++) {
                for (int k = 0; k < polylines[j].size(); k++){
                    if (j == i && (k == 0 || k == 1))                   // 跳过创建bais的点
                        continue;
                    projected_pt = current_bais_1->get_coordinate(polylines[j][k]);
                    // 将对应的基底添加入相应的网格中
                    int grid_index = get_grid_index(projected_pt);
                    if(grid_index==-1)
                        continue;
                    bais_ids_[grid_index].push_back(bais_sets_.size() - 1); // 存入当前基底的索引
                }
            }
        }
        

        // 创建第二个基底
        if(end_pt_linearity[i]>=0&&end_pt_linearity[i]<2){
            BiasPtr current_bais_2(new Bias(polylines[i][num-1], polylines[i][num-2]));
            bais_sets_.push_back(current_bais_2);
            tile_indexs_.push_back(tile_index);
            // 投影其他点到该基底坐标系下
            for (int j = 0; j < polylines.size(); j++) {
                for (int k = 0; k < polylines[j].size(); k++){
                    if (j == i && (k == num-1 || k == num-2))                   // 跳过创建bais的点
                        continue;
                    projected_pt = current_bais_2->get_coordinate(polylines[j][k]);
                    // 将对应的基底添加入相应的网格中
                    int grid_index = get_grid_index(projected_pt);
                    if(grid_index==-1)
                        continue;
                    bais_ids_[grid_index].push_back(bais_sets_.size() - 1); // 存入当前基底的索引
                }
            }
        }    
    }

    // 去除同一个网格中对应的重复的model（这是由于多个点在一个模型下映射到了同一个grid里，导致多次添加该model）
    for (int i = 0; i < bais_ids_.size(); i++){
        if(!bais_ids_[i].size())
            continue;

        int k = 0;
        for (int j = 1; j < bais_ids_[i].size(); j++)  {
            if(bais_ids_[i][k]!=bais_ids_[i][j]){
                bais_ids_[i][k + 1] = bais_ids_[i][j];
                ++k;
            }
        }
        bais_ids_[i].resize(k+1);
    }
}

// 给定基底，和折线集，计算所有点的投票结果
void GeometricHashing::Vote(const Bias &bais,
            const std::vector<std::vector<cv::Point2f> > &polylines,
            int min_vote_num_required,
            BiasPtrSet &bias_set,                                         // 存储票数不为0的bias，按票数高低排序
            std::vector<int> &vote_num,
            std::vector<int> &tile_indexs) const{                                  // 存储上述bias对应的票数
    // clear
    bias_set.clear();
    vote_num.clear();
    tile_indexs.clear();

    // voting
    std::vector<int> vote_num_all(bais_sets_.size(), 0); // 统计每个bais的票数，设置初始票数为0
    cv::Point2f projected_pt;
    int grid_index;
    for (int i = 0; i < polylines.size(); i++){
        for (int j = 0; j < polylines[i].size(); j++){
            projected_pt = bais.get_coordinate(polylines[i][j]);
            grid_index = get_grid_index(projected_pt);
            if(grid_index==-1)                                          // out of geometirc hashing bound
                continue;

            for (int k = 0; k < bais_ids_[grid_index].size(); k++){
                ++vote_num_all[bais_ids_[grid_index][k]];
            }
        }
    }

    // 搜索票数不为0的bias，并按票数排序
    std::vector<int> bias_index_set;
    for (int i = 0; i < vote_num_all.size(); i++){
        if(vote_num_all[i]>=min_vote_num_required){
            vote_num.push_back(vote_num_all[i]);
            bias_index_set.push_back(i);
        }
    }
    // 起泡排序法
    for (int i = 0; i < vote_num.size(); i++){
        for (int j = i; j < vote_num.size(); j++){
            if(vote_num[j]>vote_num[i]){
                int tmp = vote_num[i];
                vote_num[i] = vote_num[j];
                vote_num[j] = tmp;

                tmp = bias_index_set[i];
                bias_index_set[i] = bias_index_set[j];
                bias_index_set[j] = tmp;
            }
        }
    }

    for (int i = 0; i < bias_index_set.size(); i++){
        bias_set.push_back(bais_sets_[bias_index_set[i]]);
        tile_indexs.push_back(tile_indexs_[bias_index_set[i]]);
    }
}

} // namespace ghbgl 