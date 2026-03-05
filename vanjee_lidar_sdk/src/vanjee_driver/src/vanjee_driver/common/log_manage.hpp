/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Suteng Innovation Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace vanjee {
namespace lidar {
const size_t MAX_FILE_SIZE = 100 * 1024 * 1024;  // 100MB
const size_t MAX_DIR_SIZE = 1024 * 1024 * 1024;  // 1GB
const std::string LOG_DIR = (std::string)PROJECT_PATH + "/Log/";

class Logger {
 private:
  std::ofstream logFile;
  std::string currentFileName;
  size_t currentFileSize = 0;

  void openNewFile() {
    if (logFile.is_open()) {
      logFile.close();
    }
    currentFileName = generateFileName();
    logFile.open(LOG_DIR + currentFileName, std::ios::app);
    currentFileSize = getFileSize(LOG_DIR + currentFileName);
  }

  std::string generateFileName() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H.%M.%S") << ".log";
    return oss.str();
  }

  size_t getFileSize(const std::string& fileName) {
    struct stat stat_buf;
    int rc = stat(fileName.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : 0;
  }

  void checkAndCleanDirectory() {
    size_t totalSize = 0;
    std::vector<std::string> files;

    DIR* dirp = opendir(LOG_DIR.c_str());
    struct dirent* dp;
    while ((dp = readdir(dirp)) != nullptr) {
      if (dp->d_type == DT_REG) {  // Only regular files
        std::string filePath = LOG_DIR + dp->d_name;
        files.push_back(filePath);
        totalSize += getFileSize(filePath);
      }
    }
    closedir(dirp);

    if (totalSize > MAX_DIR_SIZE) {
      std::sort(files.begin(), files.end(), [](const std::string& a, const std::string& b) {
        struct stat a_stat, b_stat;
        stat(a.c_str(), &a_stat);
        stat(b.c_str(), &b_stat);
        return a_stat.st_mtime < b_stat.st_mtime;
      });

      while (totalSize > MAX_DIR_SIZE && !files.empty()) {
        totalSize -= getFileSize(files.front());
        remove(files.front().c_str());
        files.erase(files.begin());
      }
    }
  }

 public:
  Logger() {
    struct stat st = {0};
    if (stat(LOG_DIR.c_str(), &st) == -1) {
      mkdir(LOG_DIR.c_str(), 0700);
    }
    openNewFile();
    checkAndCleanDirectory();
  }

  ~Logger() {
    if (logFile.is_open()) {
      logFile.close();
    }
  }

  void log(const std::string& message, const std::string& level) {
    if (!logFile.is_open() || currentFileSize >= MAX_FILE_SIZE) {
      openNewFile();
    }

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << " [" << level << "] " << message << std::endl;
    std::string logEntry = oss.str();

    logFile << logEntry;
    logFile.flush();
    currentFileSize += logEntry.size();
  }
};
}  // namespace lidar
}  // namespace vanjee
