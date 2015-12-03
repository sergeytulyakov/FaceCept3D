#pragma once

#ifndef UIPROCESSOR_H
#define UIPROCESSOR_H

#include <Processor/TemplateCreatorProcessor.h>
#include <Grabber/GrabberBase.h>
#include <Processor/IProcessor.h>

class UIProcessor : public hpe::IProcessor
{
    public:
        UIProcessor(void);
        UIProcessor(std::string key);
        ~UIProcessor(void);

        void SetGrabber(hpe::GrabberBase *grabber);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetTemplate();

        void Process(hpe::IDataStorage::Ptr dataStorage);

    protected:
        void KeyPressed(const pcl::visualization::KeyboardEvent &event);

    private:
        hpe::GrabberBase *m_grabber;
        bool m_templateCreationTriggered;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_template;
        std::shared_ptr<hpe::TemplateCreatorProcessor> m_templateCreator;
};

#endif