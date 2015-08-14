#pragma once

#ifndef UIPROCESSOR_H
#define UIPROCESSOR_H

#include <Processor/ShowCloudProcessor.h>
#include <Processor/TemplateCreatorProcessor.h>
#include <Grabber/GrabberBase.h>
#include <Common.h>

class UIProcessor : public hpe::ShowCloudProcessor
{
    public:
        UIProcessor(void);
        UIProcessor(std::string key);
        ~UIProcessor(void);

        void SetGrabber(hpe::GrabberBase *grabber);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetTemplate();
        hpe::Common<pcl::PointXYZRGBA>::Landmarks GetLandmarks();

    protected:
        void KeyPressed(const pcl::visualization::KeyboardEvent &event);

    private:
        hpe::GrabberBase *m_grabber;
        bool m_templateCreationTriggered;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_template;
        hpe::Common<pcl::PointXYZRGBA>::Landmarks m_landmarks;
        std::shared_ptr<hpe::TemplateCreatorProcessor> m_templateCreator;
};

#endif
