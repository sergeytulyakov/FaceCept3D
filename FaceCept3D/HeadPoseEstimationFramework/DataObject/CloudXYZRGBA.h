#pragma once

#ifndef CLOUDXYZRGBA_H
#define CLOUDXYZRGBA_H

#include <DataObject/IDataObject.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hpe
{
    class CloudXYZRGBA : public IDataObject
    {
        public:
            typedef std::shared_ptr<CloudXYZRGBA> Ptr;

            CloudXYZRGBA()
            {
            }

            CloudXYZRGBA(bool create)
            {
                if (create)
                {
                    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
                }
            }

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    };
}

#endif