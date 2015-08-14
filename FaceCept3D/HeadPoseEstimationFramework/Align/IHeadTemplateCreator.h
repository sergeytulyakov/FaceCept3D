#ifndef IHEADTEMPLATECREATOR_H
#define IHEADTEMPLATECREATOR_H

#include <pcl/point_cloud.h>

#include <Interface/IInterface.h>

namespace hpe
{
    template <typename PointType>
    class IHeadTemplateCreator : public IInterface
    {
        public:
            virtual typename pcl::PointCloud<PointType>::Ptr GetTemplate() = 0;
    };
}


#endif // IHEADTEMPLATECREATOR_H
