#ifndef FILE_SYSTEM_H_
#define FILE_SYSTEM_H_
#include <vector>
#include <string>


// 获取指定目录下，所有指定类型的文件
void getFileNames(const std::string path, std::vector<std::string> &filenames, const std::string &suffix);

#endif