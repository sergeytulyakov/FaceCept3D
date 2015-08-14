#ifndef ICENTERCLOUDFILTER_H
#define ICENTERCLOUDFILTER_H

#include <Interface/IInterface.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hpe
{
    template<typename PointType>
    class ICenterCloudFilter : public IInterface
    {
        public:
            typedef pcl::PointCloud<PointType> Cloud;

            virtual typename Cloud::Ptr Filter(typename Cloud::Ptr &cloud, float x, float y, float z) = 0;
    };
}

#endif // ICENTERCLOUDFILTER_H
