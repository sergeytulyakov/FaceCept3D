#pragma once

#ifndef HEADEXTRACTIONPROCESSOR_H
#define HEADEXTRACTIONPROCESSOR_H

#include <Processor/IProcessor.h>
//#include <Landmarks.h>
#include <Filter/FilteringQueue.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hpe
{

    class HeadExtractionProcessor : public IProcessor
    {
        public:
            HeadExtractionProcessor(void);
            ~HeadExtractionProcessor(void);

            void Process(IDataStorage::Ptr storage) override;

        private:
            typedef pcl::PointXYZRGBA TPoint;
            typedef pcl::PointCloud<TPoint> TCloud;

            std::string m_cloudKey;
            std::string m_filteredCloudKey;


            bool m_readyToExtract;
            FilteringQueue<TPoint> m_filteringQueue;
    };

}

#endif