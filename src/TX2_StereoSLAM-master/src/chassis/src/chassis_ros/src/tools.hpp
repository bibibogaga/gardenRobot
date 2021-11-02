#if !defined(TOOLS_HPP)
#define TOOLS_HPP

#include <string>
#include <chrono>
#include <unistd.h> //access
#include <thread>
#include <sys/stat.h>
#include <iostream>
#include <glog/logging.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <vector>
#include <numeric>
#include <algorithm>

#ifndef RAD2DEG
#define RAD2DEG 57.2957823142244
#endif

inline std::string getCurrentSystemTime()
{
    auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm *ptm = localtime(&tt);
    char aDate[60] = {0};
    sprintf(aDate, "%d-%02d-%02d-%02d.%02d.%02d",
            (int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
            (int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
    return std::string(aDate);
}

inline bool genDir(const std::string &strDir)
{
    if (access(strDir.c_str(), 0) == -1)
    {
        const int nFlag = mkdir(strDir.c_str(), 0777);
        if (0 == nFlag)
        {
            std::cout << "make directory:" << strDir << " succeed.\n";
        }
        else
        {
            std::cout << "make directory:" << strDir << " failed.\n";
            return false;
        }
    }

    return true;
}

inline bool initGLog(const char *pPogramName, const std::string &strLogDir)
{
    if (0 != access(strLogDir.c_str(), F_OK))
    {
        if (0 != mkdir(strLogDir.c_str(), 0777))
        {
            return false;
        }
    }
    google::InitGoogleLogging(pPogramName);
    FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = 0;
    FLAGS_log_dir = strLogDir;
    FLAGS_logbufsecs = 0;
    FLAGS_logbuflevel = 0;

    return true;
}

inline geometry_msgs::Pose eigen2msg(const Eigen::Matrix4f &T)
{
    geometry_msgs::Pose pose;
    pose.position.x = T(0, 3);
    pose.position.y = T(1, 3);
    pose.position.z = T(2, 3);
    Eigen::Quaternionf quat(T.block<3, 3>(0, 0));
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    return pose;
}

inline geometry_msgs::Quaternion eigen2msg(const Eigen::Matrix3f &R)
{
    Eigen::Quaternionf quat(R);
    geometry_msgs::Quaternion orientation;
    orientation.w = quat.w();
    orientation.x = quat.x();
    orientation.y = quat.y();
    orientation.z = quat.z();
    return orientation;
}

template <typename T>
void calMeanStddev(const std::vector<T> &vData, T &mean, T &stddev)
{
    assert(vData.size() > 1);
    const T sum = std::accumulate(vData.begin(), vData.end(), T(0.0));
    mean = sum / vData.size();

    T accum = 0;
    std::for_each(std::begin(vData), std::end(vData), [&](const T d) {
        accum += (d - mean) * (d - mean);
    });
    stddev = sqrt(accum / (vData.size() - 1));
}

namespace tools
{
class Timer
{
public:
    Timer()
    {
        start_ = std::chrono::steady_clock::now();
    }
    Timer &operator=(const Timer &timer) = delete;
    Timer(const Timer &timer) = delete;
    ~Timer() = default;

    /**
     * @brief Get the Spent Time (seconds)
     * 
     * @return double 
     */
    inline double getSpentTime() const
    {
        const std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        const std::chrono::duration<double> spent = end - start_;
        return spent.count();
    }

    /**
     * @brief delay nMillSeconds millseconds
     * 
     * @param nMillSeconds 
     */
    static inline void delay(const int nMillSeconds)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(nMillSeconds));
    }

private:
    std::chrono::steady_clock::time_point start_;
};
} // namespace tools

template <typename T>
void cvtBytes2T(const uint8_t *pByte, T &val)
{
    assert(nullptr != pByte);
    val = 0;
    uint8_t *pData = (uint8_t *)&val;
    const uint8_t *pByteTmp = (const uint8_t *)pByte;
    const uint8_t nByteNum = sizeof(T);
    for (uint8_t i = 0; i < nByteNum; i++)
    {
        pData[i] = pByteTmp[i];
    }
}

template <typename T>
void cvtT2Bytes(const T &val, uint8_t *pByte)
{
    assert(nullptr != pByte);
    const uint8_t *pData = (const uint8_t *)&val;
    const uint8_t nByteNum = sizeof(T);
    for (uint8_t i = 0; i < nByteNum; i++)
    {
        pByte[i] = pData[i];
    }
}

#endif // TOOLS_HPP
