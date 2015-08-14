#ifndef BOXFILTER_H
#define BOXFILTER_H

#include "Filter/ICloudFilter.h"
#include "Filter/Filters.h"

namespace hpe
{
    /**
     \class	BoxFilter
    
     \brief	A filter that cuts a box inside a point cloud.
    
     \author	Sergey
     \date	8/11/2015
    
     \tparam	PointType	Point type of the cloud.
     */

    template<typename PointType>
    class BoxFilter : public ICloudFilter<PointType>
    {
        public:
            typedef std::shared_ptr<BoxFilter> Ptr;

            //TODO consider using pcl::BoxClipper3D here (maybe)
            BoxFilter(float xFrom, float xTo, float yFrom, float yTo, float zFrom, float zTo)
                : m_xFrom(xFrom), m_xTo(xTo), m_yFrom(yFrom), m_yTo(yTo), m_zFrom(zFrom), m_zTo(zTo)
            {
            }

            typename pcl::PointCloud<PointType>::Ptr Filter(typename pcl::PointCloud<PointType>::Ptr &cloud)
            {
                auto x = std::minmax(m_xFrom, m_xTo);
                auto c1 = PassThrough<PointType>(cloud, x.first, x.second, "x");

                auto y = std::minmax(m_yFrom, m_yTo);
                auto c2 = PassThrough<PointType>(c1, y.first, y.second, "y");

                auto z = std::minmax(m_zFrom, m_zTo);
                auto c3 = PassThrough<PointType>(c2, z.first, z.second, "z");

                return c3;
            }

        private:
            float m_xFrom;
            float m_xTo;

            float m_yFrom;
            float m_yTo;

            float m_zFrom;
            float m_zTo;
    };
}

#endif // BOXFILTER_H
