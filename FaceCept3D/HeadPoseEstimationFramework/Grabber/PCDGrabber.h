#pragma once

#include <Grabber/GrabberBase.h>
#include <DataObject/IDataObject.h>

class PCDGrabber : public hpe::GrabberBase
{
    public:
        PCDGrabber(void);
        PCDGrabber(int start, int end);
        PCDGrabber(int start, int end, std::string folder);
        PCDGrabber(std::string folder);
        ~PCDGrabber(void);

        class CloudFileInfo : public hpe::IDataObject
        {
            public:
                typedef std::shared_ptr<CloudFileInfo> Ptr;
                std::string Filename;
        };

    protected:
        void PreRun();
        hpe::IDataStorage::Ptr CreateDataStorage();
        bool GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame);

    private:
        void EnumerateFiles();

        std::vector<std::string> m_pcdFiles;

        std::string m_storeFolder;
        int m_frameNumber;
        int m_endNumber;
};

