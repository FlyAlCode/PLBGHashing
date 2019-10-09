#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "geo_locator.h"
#include "file_system.h"
#include "thin_image.h"
#include "draw_function.h"
#include "shp.h"

int main(int argc, char* argv[]){
    if(argc<5){
        std::cout << "Usage: auto_geolocalize [shp_file] [query_imgs_path] [result_file] [result_img_save_path]" << std::endl;
        return -1;
    }

    // 读取shp文件
    Shp shp_map;
    shp_map.Init(argv[1]);


    std::cout << "Building reference geometric hashing..." << std::endl;
    double t = cv::getTickCount();
    // create tile, and init geometric hashing
    // 设置geolocator的参考图，并建立geometric hashing
    ghbgl::GeometricHashingParam geometric_hashing_params;
    geometric_hashing_params.grid_size_ = 20;
    geometric_hashing_params.height_ = 10000;
    geometric_hashing_params.width_ = 10000;
    ghbgl::GeoLocator geo_locator;
    geo_locator.Init(geometric_hashing_params, 40);

    std::vector<cv::Point2f> ref_tile_offsets;
    cv::Point2f current_offset;
    cv::Mat current_ref_img;
    cv::Rect_<double> geo_bound = shp_map.get_geo_bound();
    for (int x = geo_bound.x; x < geo_bound.x + geo_bound.width; x+=1500){
        for (int y = geo_bound.y + geo_bound.height; y > geo_bound.y ; y-=1500){
            shp_map.ConvertAreaToImage(cv::Rect2d(x, y-3000, 3000, 3000), 1, current_ref_img, 3, cv::Scalar(255, 255, 255), true);
            current_offset = shp_map.GeoToImage(cv::Point2d(x, y));
            geo_locator.AddReferenceTile(current_ref_img, current_offset);
        }
    }

    // 建立geometric hashing
    geo_locator.InitGeometricHashingLocator();

    std::cout << "Finishing building geometric hashing, "<<(cv::getTickCount()-t)/cv::getTickFrequency()<<"s passed" << std::endl;

    // localization
    // 获取所有query images
    std::vector<std::string> query_img_names;
    getFileNames(argv[2], query_img_names, "png");
    cv::Mat current_query_img;
    cv::Mat current_query_img_original;
    cv::Mat current_thined_query_img;
    std::vector<cv::Mat> current_models;
    std::vector<float> current_vote_rates;

    // record result in a file
    std::ofstream fout(argv[3]);
    fout.precision(7);
    cv::Mat show_result_img;
    cv::Mat T_img_2_geo = shp_map.GetTransformationFromImageToGeo();
    for (int i = 0; i < query_img_names.size(); i++)  {
        // fout << std::endl << "Deal with: " << query_img_names[i] << "(" << i + 1 << "/" << query_img_names.size() << ")" << std::endl;
        std::cout  << std::endl << "Deal with: " << query_img_names[i] << "(" << i + 1 << "/" << query_img_names.size() << ")" << std::endl;
        current_query_img_original = cv::imread(query_img_names[i]);
        // flip(the method cannot deal flip)
        cv::flip(current_query_img_original, current_query_img, 1);
        // ThinImage(current_query_img, current_thined_query_img);

        double t = cv::getTickCount();
        geo_locator.Localize(current_query_img, 0.5, 2.0, 0.05, 0.4, current_models, current_vote_rates);

        // debug
        // std::cout << "Result for: " << query_img_names[i] << std::endl;
        // fout << "Result for: " << query_img_names[i] << std::endl;
        // 输出格式为:图片序列号 | 时间 | 合法模型的总数 | 换行 | 每行一个合理的模型
        fout << std::endl
             << i << "  " << (cv::getTickCount() - t) / cv::getTickFrequency() << "  ";
        fout << current_models.size() << std::endl;

        bool draw_success = false;
        float flip_offset_data[9] = {-1, 0, current_query_img_original.cols, 0, 1, 0, 0, 0, 1};
        cv::Mat flip_offset(3, 3, CV_32F, flip_offset_data);
        for (int j = 0; j < current_models.size(); j++) {
            std::cout << "vote_num = " << current_vote_rates[j] << std::endl;
            // std::cout << current_models[j] << std::endl;

            // fout << j << ": vote_num = " << current_vote_rates[j] << std::endl;
            // fout << current_models[j] << std::endl;
            fout << current_vote_rates[j] << "   ";
            cv::Mat T_2_geo = T_img_2_geo * current_models[j] * flip_offset;
            for (int x = 0; x < 3; x++) {
                for (int y = 0; y < 3; y++){
                    fout << T_2_geo.at<float>(x, y) << "  ";
                }
            }
            fout << std::endl;

            if (!draw_success)  {
                draw_success = shp_map.ShowOnMap(current_query_img_original, T_2_geo, 100, cv::Scalar(255, 0, 255), show_result_img);
                if(draw_success){
                    std::cout << j << ": vote_num = " << current_vote_rates[j] << std::endl;

                    char save_name[50];
                    sprintf(save_name, "%.3d_%.3d.png", i, j);
                    cv::imwrite(std::string(argv[4]) + save_name, show_result_img);

                    char key;
                    cv::imshow("result", show_result_img);
                    key = cv::waitKey(30);
                    // if(key == 'q')
                        // break;
                }
            }
        }
    }
}