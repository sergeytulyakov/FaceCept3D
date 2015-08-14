#ifndef KINECTDATACONVERTER_H
#define KINECTDATACONVERTER_H

#include <Converter/IDataConverter.h>

namespace hpe
{
    /**
     \class	KinectDataConverter
    
     \brief	Converts RGB-D pair to a point cloud. Work only for a kinect sensor.
    
     \author	Sergey
     \date	8/11/2015
     */

    class KinectDataConverter : public IDataConverter
    {
        public:
            KinectDataConverter();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr DepthRGBToPointCloud(cv::Mat &depth, cv::Mat &bgr) override;
            pcl::PointXYZ ConvertOnePoint(float x, float y, float depthValue, cv::Size frameSize);

        protected:
            virtual void ComputeCameraIntrinsics(cv::Size &imageSize, float &centerX, float &centerY, float &scaleFactorX, float &scaleFactorY);
    };
}

#endif // KINECTDATACONVERTER_H
