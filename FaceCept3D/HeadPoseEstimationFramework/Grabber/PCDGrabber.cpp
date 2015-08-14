#include "PCDGrabber.h"

#include <DataObject/MapDataStorage.h>
#include <DataObject/CloudXYZRGBA.h>
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>

PCDGrabber::PCDGrabber(void)
    : m_frameNumber(0), m_endNumber(100), m_storeFolder("clouds")
{
}

PCDGrabber::PCDGrabber(int start, int end)
    : m_frameNumber(start), m_endNumber(end), m_storeFolder("clouds")
{
}

PCDGrabber::PCDGrabber(std::string folder)
    : m_frameNumber(0), m_endNumber(0), m_storeFolder(folder)
{
}

PCDGrabber::PCDGrabber(int start, int end, std::string folder)
    : m_frameNumber(start), m_endNumber(end), m_storeFolder(folder)
{
}

PCDGrabber::~PCDGrabber(void)
{
}

void PCDGrabber::PreRun()
{
    m_frameNumber = 1;
    EnumerateFiles();
}

hpe::IDataStorage::Ptr PCDGrabber::CreateDataStorage()
{
    using namespace hpe;
    IDataStorage::Ptr result(new MapDataStorage);
    CloudXYZRGBA::Ptr cloudObj(new CloudXYZRGBA(true));
    std::string path = m_pcdFiles[m_frameNumber];
    m_frameNumber += 1;
    pcl::io::loadPCDFile(path, *(cloudObj->cloud));
    result->Set("Cloud", cloudObj);

    CloudFileInfo::Ptr fileinfo(new CloudFileInfo);
    fileinfo->Filename = path;
    result->Set("FileInfo", fileinfo);

    return result;
}

bool PCDGrabber::GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame)
{
    return m_frameNumber < m_pcdFiles.size();
}

struct SortNumerically
{
    template <typename T>
    T st2num(const std::string &Text)
    {
        std::stringstream ss(Text);
        T result;
        return ss >> result ? result : 0;
    }

    bool operator()(const std::string &a, const std::string &b)
    {
        namespace fs = boost::filesystem;
        fs::path p1 = fs::path(a);
        fs::path p2 = fs::path(b);

        int v1 = st2num<int>(p1.stem().string());
        int v2 = st2num<int>(p2.stem().string());

        return v1 < v2;
    }
};

void PCDGrabber::EnumerateFiles()
{
    namespace fs = boost::filesystem;

    fs::path rootFolder(m_storeFolder);

    if (fs::exists(rootFolder))
    {
        if (fs::is_directory(rootFolder))
        {
            auto it = fs::directory_iterator(rootFolder);
            for (; it != fs::directory_iterator(); it++)
            {
                auto fileName = *it;
                if (fs::is_regular(fileName))
                {
                    auto path = fileName.path();
                    fs::path extension = path.extension();
                    auto filename = path.filename().string();
                    std::string p = path.string();
                    if (extension == ".pcd")
                    {
                        m_pcdFiles.push_back(p);
                    }
                }
            }
        }
    }

    SortNumerically s;
    std::sort(m_pcdFiles.begin(), m_pcdFiles.end(), s);
}