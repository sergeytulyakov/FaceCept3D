#ifndef CENTEREDBOXFILTER_H
#define CENTEREDBOXFILTER_H

#include "Filter/ICenterCloudFilter.h"
#include "Filter/Filters.h"

#include <pcl/point_cloud.h>

namespace hpe
{
    template <typename PointT>
    class CenteredBoxFilter : public ICenterCloudFilter<PointT>
    {
        public:
            typedef pcl::PointCloud<PointT> Cloud;

            CenteredBoxFilter(float sizeX, float sizeY, float sizeZ)
                : m_sizeX(sizeX / 2), m_sizeY(sizeY / 2), m_sizeZ(sizeZ / 2)
            {

            }

            typename Cloud::Ptr Filter(typename Cloud::Ptr &cloud, float x, float y, float z)
            {
                float xFrom = x - m_sizeX;
                float xTo = x + m_sizeX;
                auto result = PassThrough<PointT>(cloud, xFrom, xTo, "x", cloud->isOrganized());

                float yFrom = y - m_sizeY;
                float yTo = y + m_sizeY;
                result = PassThrough<PointT>(result, yFrom, yTo, "y", cloud->isOrganized());

                float zFrom = z - m_sizeZ;
                float zTo = z + m_sizeZ;
                result = PassThrough<PointT>(result, zFrom, zTo, "z", cloud->isOrganized());

                return result;
            }


        private:
            float m_sizeX;
            float m_sizeY;
            float m_sizeZ;
    };
}

#endif // CENTEREDBOXFILTER_H
