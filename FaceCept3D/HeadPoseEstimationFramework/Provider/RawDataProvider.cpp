#include "RawDataProvider.h"

#include <Provider/CvMatRawSerializer.h>
#include <Converter/KinectDataConverter.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

namespace fs = boost::filesystem;

namespace hpe
{
    RawDataProvider::RawDataProvider()
        : m_initialized(false)
    {
    }

    cv::Mat RawDataProvider::GetDepth(int frameNumber)
    {
        CvMatRawSerializer reader;
        cv::Mat depthImage;
        reader.Load(m_depthImageFiles[frameNumber], depthImage);
        return depthImage;
    }

    cv::Mat RawDataProvider::GetBgr(int frameNumber)
    {
        return cv::imread(m_imageFiles[frameNumber]);
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RawDataProvider::GetCloud(int frameNumber)
    {
        CvMatRawSerializer reader;
        cv::Mat depthImage;
        reader.Load(m_depthImageFiles[frameNumber], depthImage);

        cv::Mat image = cv::imread(m_imageFiles[frameNumber]);

        KinectDataConverter converter;
        return converter.DepthRGBToPointCloud(depthImage, image);
    }

    RawDataProvider::Landmarks RawDataProvider::GetPoints(int frameNumber)
    {
        RawDataProvider::Landmarks landmarks;

        if (frameNumber < m_pointsFiles.size() || m_pointsFiles.empty())
        {
            return landmarks;
        }

        std::string landmarkFile = m_pointsFiles[frameNumber];
        std::ifstream stream(landmarkFile.c_str());
        std::string line;
        while (std::getline(stream, line))
        {
            boost::tokenizer<boost::escaped_list_separator<char>> tokenizer(line);
            std::vector<std::string> tokens;
            tokens.assign(tokenizer.begin(), tokenizer.end());

            RawDataProvider::Landmarks::value_type landmark;
            std::stringstream sstream(tokens[0]);
            int x, y;
            sstream >> x;
            sstream.str(tokens[1]);
            sstream.clear();
            sstream >> y;

            auto cloud = GetCloud(frameNumber);

            landmark.index = (int)x + cloud->width * (int)y;
            landmark.point = cloud->at(landmark.index);
            landmarks.push_back(landmark);
        }

        return landmarks;
    }

    int RawDataProvider::GetFrameCount()
    {
        int m1 = std::min(m_depthImageFiles.size(), m_imageFiles.size());
        return m1;
        return std::min(m1, m_numberOfFrames);
    }

    int RawDataProvider::GetFrequency()
    {
        return m_frequency;
    }

    void RawDataProvider::CheckInitialized()
    {
        if (m_initialized == false)
        {

        }
    }

    void RawDataProvider::SetFolder(const std::string &path)
    {
        m_pointCloudFolder = path;

        if (fs::exists(m_pointCloudFolder) == false)
        {
            throw std::invalid_argument("DataProvider::SetFolder: pointCloudFolder does not exist");
        }

        std::string videoPath = (fs::path(m_pointCloudFolder) / "video.mpg").string();

        if (fs::exists(videoPath) == false)
        {
            //throw std::invalid_argument("DataProvider::SetFolder: video does not exist");
        }

        m_videoCapture = std::shared_ptr<cv::VideoCapture>(new cv::VideoCapture);
        m_videoCapture->open(videoPath);

        EnumerateFiles();

        m_initialized = true;
    }

    void RawDataProvider::EnumerateFiles()
    {
        fs::path rootFolder(m_pointCloudFolder);

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
                        if (filename == "shimmeroutput.csv")
                        {
                            continue;
                        }
                        std::string p = path.string();
                        if (extension == ".png")
                        {
                            m_imageFiles.push_back(p);
                        }
                        else if (extension == ".raw")
                        {
                            m_depthImageFiles.push_back(p);
                        }
                        else if (extension == ".csv")
                        {
                            m_pointsFiles.push_back(p);
                        }
                    }
                }
            }
        }

        std::sort(m_imageFiles.begin(), m_imageFiles.end());
        std::sort(m_depthImageFiles.begin(), m_depthImageFiles.end());
        std::sort(m_pointsFiles.begin(), m_pointsFiles.end());
    }
}

