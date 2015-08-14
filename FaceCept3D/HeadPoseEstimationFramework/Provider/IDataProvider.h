#ifndef IDATAPROVIDER_H
#define IDATAPROVIDER_H

#include <Interface/IInterface.h>
#include <Common.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>


namespace hpe
{
    /**
     \class	IDataProvider
    
     \brief	An interface that provides access to RGB-D pairs, point clouds, and 
			landmarks if available. Can be used to imitate grabbing from a sensor.
    
     \author	Sergey
     \date	8/11/2015
    
     \tparam	PointType	Type of the point type.
     */

    template <typename PointType>
    class IDataProvider : public IInterface
    {
        public:
            virtual int GetFrameCount() = 0;
            virtual cv::Mat GetDepth(int frameNumber) = 0;
            virtual cv::Mat GetBgr(int frameNumber) = 0;
            virtual typename pcl::PointCloud<PointType>::Ptr GetCloud(int frameNumber) = 0;
            virtual typename Common<PointType>::Landmarks GetPoints(int frameNumber) = 0;
    };
}



#endif // IDATAPROVIDER_H
