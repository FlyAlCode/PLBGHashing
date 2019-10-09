#include "file_system.h"
#include <iostream>
#include <fstream>
#include <string.h> //包含strcmp的头文件,也可用: #include <ctring>
#include <dirent.h>
#include <algorithm>

void getFileNames(const std::string path, std::vector<std::string>& filenames, const std::string &suffix){    
    DIR *pDir;    
    struct dirent* ptr;    
    if (!(pDir = opendir(path.c_str())))        
        return;
    
    while ((ptr = readdir(pDir))!=0) {        
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){            
            std::string file = path + "/" + ptr->d_name;            
            if (opendir(file.c_str()))  {                
                getFileNames(file, filenames, suffix);                
            }
            
            else {                
                if (suffix == file.substr(file.size() - suffix.size())){                    
                    filenames.push_back(file);                    
                }                
            }            
        }        
    }    
    closedir(pDir);

    std::sort(filenames.begin(), filenames.end());
}