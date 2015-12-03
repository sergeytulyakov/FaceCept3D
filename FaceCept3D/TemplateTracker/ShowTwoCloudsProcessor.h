#pragma once

#ifndef SHOWTWOCLOUDSPROCESSOR_H
#define SHOWTWOCLOUDSPROCESSOR_H

#include <Processor/IProcessor.h>
#include <pcl/visualization/pcl_visualizer.h>

class ShowTwoCloudsProcessor : public hpe::IProcessor
{
    public:
        ShowTwoCloudsProcessor(void);
        ~ShowTwoCloudsProcessor(void);

        void SetCloud1Key(std::string key);
        void SetCloud2Key(std::string key);

        void Process(hpe::IDataStorage::Ptr dataStorage);

    private:
        std::string m_cloud1Key;
        std::string m_cloud2Key;
        pcl::visualization::PCLVisualizer m_visualizer;
        bool m_first;
};

#endif