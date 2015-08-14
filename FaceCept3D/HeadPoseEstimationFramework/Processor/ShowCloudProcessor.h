#pragma once

#ifndef SHOWCLOUDPROCESSOR_H
#define SHOWCLOUDPROCESSOR_H

#include <Processor/IProcessor.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>

namespace hpe
{

    class ShowCloudProcessor : public IProcessor
    {
        public:
            ShowCloudProcessor(void);
            ShowCloudProcessor(std::string key);
            ~ShowCloudProcessor(void);

            void Process(IDataStorage::Ptr storage);

        protected:
            virtual void KeyPressed(const pcl::visualization::KeyboardEvent &event);

        private:
            static void KeyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void *sender);

            std::string m_key;
            pcl::visualization::PCLVisualizer m_visualizer;
            bool m_first;
    };

}

#endif