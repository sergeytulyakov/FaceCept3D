#include "SaveCloudProcessor.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <DataObject/CloudXYZRGBA.h>
#include <DataObject/RawFrames.h>

#include <opencv/highgui.h>

namespace hpe
{

    SaveCloudProcessor::SaveCloudProcessor(void)
        : m_frameNumber(0), m_dir("clouds"), m_running(true)
    {
    }

    SaveCloudProcessor::~SaveCloudProcessor(void)
    {
        m_videoWriter->release();
    }

    void SaveCloudProcessor::Init()
    {
        m_writingThread = boost::thread(boost::bind(&SaveCloudProcessor::SavingThreadRoutine, this));

        namespace pt = boost::posix_time;
        m_dir = pt::to_iso_string(pt::second_clock::local_time());
        boost::filesystem::create_directory(m_dir);

    }

    void SaveCloudProcessor::ProcessEntry(Entry entry)
    {
        if (m_videoWriter.get() == nullptr)
        {
            m_videoWriter = std::shared_ptr<cv::VideoWriter>(new cv::VideoWriter);
            m_videoWriter->open((boost::format("%1%/video.mpg") % m_dir).str(), CV_FOURCC('M', 'P', 'E', 'G'), 30, cv::Size(640, 480));
        }

        std::vector<cv::Mat> channels;
        cv::split(entry.frame, channels);
        channels.erase(channels.begin() + 3);
        cv::Mat colorWithoutAlpha;

        cv::merge(channels, colorWithoutAlpha);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*(entry.cloud), *(entry.cloud), indices);

        std::string savePathCloud = (boost::format("%1%/%2%.pcd") % m_dir % m_frameNumber).str();
        m_frameNumber += 1;

        m_videoWriter->write(colorWithoutAlpha);
        pcl::io::savePCDFileBinary(savePathCloud, *(entry.cloud));
    }

    void SaveCloudProcessor::SavingThreadRoutine()
    {
        while (m_running)
        {
            Entry entry;
            m_mutex.lock();
            if (m_entries.size() != 0)
            {
                entry = m_entries.front();
                m_entries.pop();
                m_mutex.unlock();

                ProcessEntry(entry);
            }
            else
            {
                m_mutex.unlock();
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
        }

        m_mutex.lock();

        while (m_entries.size() != 0)
        {
            ProcessEntry(m_entries.front());
            m_entries.pop();
        }
        m_videoWriter->release();

        m_mutex.unlock();
    }

    void SaveCloudProcessor::Cleanup()
    {
        m_running = false;
        m_writingThread.join();
    }

    void hpe::SaveCloudProcessor::Process(IDataStorage::Ptr dataStorage)
    {
        CloudXYZRGBA::Ptr cloudObject = dataStorage->GetAndCast<CloudXYZRGBA>("Cloud");
        RawFrames::Ptr frames = dataStorage->GetAndCast<RawFrames>("RawFrames");
        if (cloudObject.get() == nullptr || frames.get() == nullptr)
        {
            return;
        }

        Entry entry;
        frames->colorFrame.copyTo(entry.frame);
        pcl::copyPointCloud(*(cloudObject->cloud), *(entry.cloud));

        m_mutex.lock();
        m_entries.push(entry);
        m_mutex.unlock();
    }

}