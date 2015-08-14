#pragma once

#ifndef MOVINGLEASTSQUARESFILTER_H
#define MOVINGLEASTSQUARESFILTER_H

#include <Filter/ICloudFilter.h>

#include <pcl/search/kdtree.h>
//#define PCL_NO_PRECOMPILE
#include <pcl/surface/mls.h>
//#undef PCL_NO_PRECOMPILE

namespace hpe
{
    template <class PointType>
    class MovingLeastSquaresFilter : public ICloudFilter<PointType>
    {
        public:
            typedef typename ICloudFilter<PointType>::Cloud Cloud;

            typename Cloud::Ptr Filter(typename Cloud::Ptr &cloud)
            {
                typename Cloud::Ptr result(new Cloud);

                typename pcl::search::KdTree<PointType>::Ptr search(new pcl::search::KdTree<PointType>);
                typename pcl::MovingLeastSquares<PointType, PointType> mls;

                search->setInputCloud(cloud);

                mls.setInputCloud(cloud);
                mls.setPolynomialFit(true);
                mls.setPolynomialOrder(3);
                mls.setSearchMethod(search);
                mls.setSearchRadius(0.015);
                mls.setUpsamplingMethod(mls.VOXEL_GRID_DILATION);
                mls.setDilationVoxelSize(0.001);

                mls.process(*result);

                return result;
            }
    };
}

#endif
