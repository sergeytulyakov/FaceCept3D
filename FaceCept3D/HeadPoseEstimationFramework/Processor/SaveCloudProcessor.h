#pragma once

#ifndef SAVECLOUDPROCESSOR_H
#define SAVECLOUDPROCESSOR_H

#include <Processor/IProcessor.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <queue>

namespace hpe
{

    class SaveCloudProcessor : public IProcessor
    {
        public:
            SaveCloudProcessor(void);
            ~SaveCloudProcessor(void);

            void Process(IDataStorage::Ptr dataStorage);
            void Init();
            void Cleanup();
        private:
            struct Entry
            {
                Entry() : cloud(new pcl::PointCloud<pcl::PointXYZRGBA>)
                {}
                cv::Mat frame;
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
            };

            int m_frameNumber;
            std::string m_dir;
            std::shared_ptr<cv::VideoWriter> m_videoWriter;

            boost::mutex m_mutex;
            std::queue<Entry> m_entries;
            boost::thread m_writingThread;
            bool m_running;

            void SavingThreadRoutine();
            void ProcessEntry(Entry entry);
    };

}

#endif