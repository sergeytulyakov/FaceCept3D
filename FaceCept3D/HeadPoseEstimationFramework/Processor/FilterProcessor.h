#pragma once

#ifndef FILTERPROCESSOR_H
#define FILTERPROCESSOR_H

#include <Processor/IProcessor.h>
#include <Filter/ICloudFilter.h>

namespace hpe
{
    class FilterProcessor : public IProcessor
    {
        public:
            typedef std::shared_ptr<ICloudFilter<pcl::PointXYZRGBA>> FilterPtr;

            FilterProcessor(FilterPtr filter, std::string inputKey, std::string outputKey);

            void Process(IDataStorage::Ptr dataStorage);

        private:
            FilterPtr m_filter;
            std::string m_inputKey;
            std::string m_outputKey;
    };

}


#endif