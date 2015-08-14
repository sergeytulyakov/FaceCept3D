#include "ShowCloudProcessor.h"

#include <DataObject/CloudXYZRGBA.h>
#include <pcl/common/time.h>

namespace hpe
{

    ShowCloudProcessor::ShowCloudProcessor(void)
        : m_key("Cloud"), m_visualizer("Cloud"), m_first(true)
    {
    }

    ShowCloudProcessor::ShowCloudProcessor(std::string key)
        : m_key(key), m_visualizer(key), m_first(true)
    {
    }

    ShowCloudProcessor::~ShowCloudProcessor(void)
    {
    }

    void ShowCloudProcessor::Process(IDataStorage::Ptr storage)
    {
        IDataObject::Ptr obj = storage->Get(m_key);
        CloudXYZRGBA::Ptr cloudObject = std::dynamic_pointer_cast<CloudXYZRGBA>(obj);
        if (cloudObject.get() == nullptr)
        {
            return;
        }

        if (m_first)
        {
            m_visualizer.registerKeyboardCallback(&ShowCloudProcessor::KeyboardEventCallback, this);
            m_visualizer.addPointCloud(cloudObject->cloud, m_key);
            m_first = false;
        }
        else
        {
            m_visualizer.updatePointCloud(cloudObject->cloud, m_key);
        }

        m_visualizer.spinOnce();
    }

    void ShowCloudProcessor::KeyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void *sender)
    {
        ShowCloudProcessor *_this = static_cast<ShowCloudProcessor *>(sender);
        _this->KeyPressed(event);
    }

    void ShowCloudProcessor::KeyPressed(const pcl::visualization::KeyboardEvent &event)
    {
    }

}