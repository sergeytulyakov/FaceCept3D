#ifndef FUNCTORFILTER_H
#define FUNCTORFILTER_H

#include <functional>

#include <pcl/point_cloud.h>
#include "Filter/ICloudFilter.h"

namespace hpe
{
    /**
     \class	FunctorFilter
    
     \brief	A filter that performes the operation defined inside a functor. Clearly
			the functor should conform to Functor format.
    
     \author	Sergey
     \date	8/11/2015
    
     \tparam	PointType	Type of the point type.
     */

    template<typename PointType>
    class FunctorFilter : public ICloudFilter<PointType>
    {
        public:
            typedef typename std::function<typename pcl::PointCloud<PointType>::Ptr(typename pcl::PointCloud<PointType>::Ptr &)> Functor;
            typedef std::shared_ptr<FunctorFilter> Ptr;

            FunctorFilter(Functor functor)
                : m_functor(functor)
            {

            }

            typename pcl::PointCloud<PointType>::Ptr Filter(typename pcl::PointCloud<PointType>::Ptr &cloud)
            {
                return m_functor(cloud);
            }

        private:
            Functor m_functor;

    };
}

#endif // FUNCTORFILTER_H
