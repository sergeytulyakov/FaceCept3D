#ifndef RAWDATAPROVIDER_H
#define RAWDATAPROVIDER_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include "Provider/IDataProvider.h"
#include "Common.h"

namespace hpe
{
    class RawDataProvider : public IDataProvider<pcl::PointXYZRGBA>
    {
        public:
            typedef Common<pcl::PointXYZRGBA>::Landmarks Landmarks;

            RawDataProvider();

            void SetFolder(const std::string &path);

            cv::Mat GetDepth(int frameNumber);
            cv::Mat GetBgr(int frameNumber);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetCloud(int frameNumber);
            Landmarks GetPoints(int frameNumber);

            int GetFrameCount();
            int GetFrequency();

        private:
            void CheckInitialized();
            void EnumerateFiles();

            std::vector<std::string> m_imageFiles;
            std::vector<std::string> m_depthImageFiles;
            std::vector<std::string> m_pointsFiles;


            std::string m_pointCloudFolder;

            int m_numberOfFrames;
            int m_frequency;

            bool m_initialized;

            std::shared_ptr<cv::VideoCapture> m_videoCapture;
    };
}

#endif // DATAPROVIDER_H
