#ifndef ICLOUDFILTER_H
#define ICLOUDFILTER_H

#include <Interface/IInterface.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hpe
{
    /**
     \class	ICloudFilter
    
     \brief	A generic interface that takes a cloud and produces another cloud.
    
     \author	Sergey
     \date	8/11/2015
    
     \tparam	PointType	Type of the point type.
     */

    template<typename PointType>
    class ICloudFilter : public IInterface
    {
        public:
            typedef pcl::PointCloud<PointType> Cloud;

            virtual typename Cloud::Ptr Filter(typename Cloud::Ptr &cloud) = 0;
    };
}

#endif // ICLOUDFILTER_H
