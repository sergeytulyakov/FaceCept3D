#include "DepthPreprocessingProcessor.h"
#include <DataObject/RawFrames.h>
#include <DataObject/CloudXYZRGBA.h>
#include <Exception/HPEException.h>
#include <opencv2/opencv.hpp>
namespace hpe
{

    DepthPreprocessingProcessor::DepthPreprocessingProcessor(void)
        : m_framesKey("RawFrames"), m_erodeSize(9), m_closingSize(20), m_gaussianSize(7), m_gaussianSigma(7), m_distanceThreshold(1.2), m_saveOriginalCloud(false)
    {
    }

    DepthPreprocessingProcessor::DepthPreprocessingProcessor(int erodeSize, int closingSize, int gaussianSize, float gaussianSigma, float threshold)
        : m_framesKey("RawFrames"), m_erodeSize(erodeSize), m_closingSize(closingSize), m_gaussianSize(gaussianSize), m_gaussianSigma(gaussianSigma), m_distanceThreshold(threshold), m_saveOriginalCloud(false)
    {

    }

    DepthPreprocessingProcessor::~DepthPreprocessingProcessor(void)
    {
    }

    void DepthPreprocessingProcessor::Process(IDataStorage::Ptr storage)
    {
        RawFrames::Ptr frames = storage->GetAndCastNotNull<RawFrames>(m_framesKey);

        if (m_saveOriginalCloud)
        {
            CloudXYZRGBA::Ptr cloudObject(new CloudXYZRGBA);
            cloudObject->cloud = m_converter->DepthRGBToPointCloud(frames->depthFrame, frames->colorFrame);
            storage->Set("OriginalCloud", cloudObject);
        }

        cv::Mat mask = frames->depthFrame < m_distanceThreshold;
        cv::Mat convolved = AndMask(frames->depthFrame, mask);
        cv::GaussianBlur(convolved, convolved, cv::Size(m_gaussianSize, m_gaussianSize), m_gaussianSigma, m_gaussianSigma, cv::BORDER_DEFAULT);

        cv::Mat mask2;
        cv::Mat closingKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_closingSize, m_closingSize));
        cv::morphologyEx(mask, mask2, cv::MORPH_CLOSE, closingKernel);

        cv::Mat erodeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_erodeSize, m_erodeSize));
        cv::Mat realMask;
        cv::morphologyEx(mask2, realMask, cv::MORPH_ERODE, erodeKernel);
        cv::Mat bodyExtracted = AndMask(convolved, realMask);

        bodyExtracted.copyTo(frames->depthFrame);
    }

    cv::Mat DepthPreprocessingProcessor::AndMask(cv::Mat &matrix, cv::Mat &mat)
    {
        if (matrix.size() != mat.size())
        {
            throw HPEException("cv::Mat AndMask(cv::Mat &matrix, cv::Mat &mat) - matrix.size() != mat.size()");
        }

        cv::Mat result(matrix.rows, matrix.cols, matrix.type());

        cv::MatIterator_<float> target = result.begin<float>();
        cv::MatIterator_<float> source = matrix.begin<float>();

        cv::MatIterator_<uchar> maskIterator = mat.begin<uchar>();

        for (; target != result.end<float>(); ++target, ++source, ++ maskIterator)
        {
            if (*maskIterator != 0)
            {
                *target = *source;
            }
            else
            {
                *target = std::numeric_limits<float>::quiet_NaN();
            }
        }

        return result;
    }

    void DepthPreprocessingProcessor::SetConverter(ConverterPtr c)
    {
        m_converter = c;
        m_saveOriginalCloud = true;
    }

}