#pragma once

#ifndef FILTERINGQUEUE_H
#define FILTERINGQUEUE_H

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include "Filter/ICloudFilter.h"
#include "Filter/FunctorFilter.h"
#include <vector>


namespace hpe
{
	/**
	 \class	FilteringQueue
	
	 \brief	A queue of ICloudFilter objects that has ICloudFilter interface.
	
	 \author	Sergey
	 \date	8/11/2015
	
	 \tparam	PointType	Type of the point type.
	 */

	template<typename PointType>
    class FilteringQueue : public ICloudFilter<PointType>
    {
        public:
            typedef std::shared_ptr<FilteringQueue> Ptr;
            typedef std::shared_ptr<ICloudFilter<PointType>> InputFilterPtr;

            FilteringQueue() {}

            void AddFilter(std::shared_ptr<ICloudFilter<PointType>> filter)
            {
                m_filters.push_back(filter);
            }

            void AddFilterFunctor(typename FunctorFilter<PointType>::Functor functor)
            {
                typename FunctorFilter<PointType>::Ptr filter(new FunctorFilter<PointType>(functor));
                m_filters.push_back(filter);
            }

            typename pcl::PointCloud<PointType>::Ptr Filter(typename pcl::PointCloud<PointType>::Ptr &cloud)
            {
                typename pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>);
                pcl::copyPointCloud(*cloud, *result);
                for (auto it = m_filters.begin(); it != m_filters.end(); ++it)
                {
                    result = (*it)->Filter(result);
                }

                return result;
            }

        private:
            std::vector<typename std::shared_ptr<ICloudFilter<PointType>>> m_filters;
    };
}

#endif // FILTERINGQUEUE_H
