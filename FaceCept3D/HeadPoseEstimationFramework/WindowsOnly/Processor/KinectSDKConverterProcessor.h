#ifndef KINECTSDKCONVERTERPROCESSOR_H
#define KINECTSDKCONVERTERPROCESSOR_H

#include <Processor/IProcessor.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Windows.h>

namespace hpe
{
    class KinectSDKConverterProcessor : public IProcessor
    {
        public:
            KinectSDKConverterProcessor();
            KinectSDKConverterProcessor(std::string key);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr DepthRGBToPointCloud(cv::Mat &depth, cv::Mat &bgr);
            void Process(IDataStorage::Ptr dataStorage) override;
        private:
            void ComputeCameraIntrinsics(cv::Size &imageSize, float &centerX, float &centerY, float &scaleFactorX, float &scaleFactorY);

            LONG *m_correspondencies;
            std::string m_cloudKey;
    };
}

#endif // KINECTDATACONVERTER_H
