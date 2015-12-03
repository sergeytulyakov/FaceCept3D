#include "FacialExpressionsProcessor.h"

#include <DataObject/CloudXYZRGBA.h>
#include <DataObject/LandmarksObject.h>

#include <UI/ShowCloud.h>

#include <Helpers/HpeHelpers.h>

#include <boost/format.hpp>

namespace hpe
{
    void FacialExpressionProcessor::Process(IDataStorage::Ptr dataStorage)
    {
        if (m_dataReady)
        {
            m_facialExpressionReady(m_ferResult);
            m_dataReady = false;
        }

        if (m_workerBusy == false)
        {
            CloudXYZRGBA::Ptr cloudObject = dataStorage->GetAndCast<CloudXYZRGBA>("OriginalCloud");
            if (cloudObject.get() == nullptr)
            {
                cloudObject = dataStorage->GetAndCastNotNull<CloudXYZRGBA>("Cloud", "FacialExpressionProcessor::Process - cloud is null");
            }
            LandmarksObject<pcl::PointXYZRGBA>::Ptr landmarksObject = dataStorage->GetAndCastNotNull<LandmarksObject<pcl::PointXYZRGBA>>("Landmarks", "FacialExpressionProcessor::Process - landmarks are null");

            Common<PointT>::Landmarks l;
            for (int i = 0; i < landmarksObject->landmarks.size(); i += 1)
            {
                PointT pt;
                pt.getVector3fMap() = landmarksObject->landmarks[i].point.getVector3fMap();
                l.push_back(Common<PointT>::Landmark(landmarksObject->landmarks[i].index, pt));
            }

            m_landmarks = l;
            pcl::copyPointCloud(*(cloudObject->cloud), m_cloud);

            m_workerBusy = true;
        }

    }

    std::shared_ptr<CylinderSampler> FacialExpressionProcessor::CreateCylindricalSampler()
    {
        /*
        PhiRange=0.05,0.95
        ZRange=1.3,0.9
        TopRows=45
        BottomRows=105
        SampleColumns=120
        */
        CylinderSampler::Range phiRange;
        phiRange.first = 0.05;
        phiRange.second = 0.95;

        CylinderSampler::Range zRange;
        zRange.first = 0.9;
        zRange.second = 1.3;

        int topRows = 45;
        int bottomRows = 105;
        int sampleColumns = 120;

        std::vector<int> leftEyeIndices;
        leftEyeIndices.push_back(0);
        std::vector<int> rightEyeIndices;
        rightEyeIndices.push_back(1);

        std::shared_ptr<CylinderSampler> sampler(new CylinderSampler(phiRange, zRange, topRows, bottomRows, sampleColumns));
        sampler->SetVisualize(false);
        sampler->SetSupportOptimizedSampling(true);
        sampler->SetEyeIndices(leftEyeIndices, rightEyeIndices);

        return sampler;
    }

    FacialExpressionProcessor::FacialExpressionProcessor()
    {
        Init("FERData");
    }

    FacialExpressionProcessor::FacialExpressionProcessor(std::string dataFolder)
    {
        Init(dataFolder);
    }

    //called in constructors
    void FacialExpressionProcessor::Init(std::string dataFolder)
    {
        m_workerBusy = false;
        m_dataReady = false;

        std::string paramsDir = (boost::format("%s/params/") % dataFolder).str();
        std::string forestDir = (boost::format("%s/trees/") % dataFolder).str();

        fer::RV_readParamList(paramsDir, &m_ferParameters);

        m_forests = fer::RV_readAllForests(forestDir, m_ferParameters);

        m_numberOfClasses = 3;
    }


    //called before grabber starts to send frames
    void FacialExpressionProcessor::Init()
    {
        std::shared_ptr<hpe::CylinderSampler> sampler = CreateCylindricalSampler();
        m_featureCalculator = std::shared_ptr<hpe::CylinderOptimizedFeatureCalculator<PointT, NormalT>>(new hpe::CylinderOptimizedFeatureCalculator<PointT, NormalT>(sampler, hpe::FeatureType::Depth));
        m_featureCalculator->SetFeatureSize(cv::Size(120, 150));

        boost::thread t(boost::bind(&FacialExpressionProcessor::ThreadRoutine, this));
    }

    void FacialExpressionProcessor::ThreadRoutine()
    {
        while (true)
        {
            while (m_workerBusy == false)
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            }

            auto l = m_landmarks;

            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

            Eigen::Vector3f zAxis;
            zAxis << 0, 0, 1;
            Eigen::Vector3f normal = l[3].point.getVector3fMap() - l[2].point.getVector3fMap();

            Eigen::Quaternionf rotation;
            rotation.setFromTwoVectors(normal, zAxis);
            Eigen::Matrix4f transformRotation = Eigen::Matrix4f::Identity();
            transformRotation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            pcl::transformPointCloud(m_cloud, *cloud, transformRotation);

            l = TransformLandmarks<PointT>(l, transformRotation);

            Eigen::Vector3f xAxis;
            xAxis << 1, 0, 0;
            Eigen::Vector3f eyesLine = l[1].point.getVector3fMap() - l[0].point.getVector3fMap();

            rotation.setFromTwoVectors(eyesLine, xAxis);
            transformRotation = Eigen::Matrix4f::Identity();
            transformRotation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            pcl::transformPointCloud(*cloud, *cloud, transformRotation);

            l = TransformLandmarks<PointT>(l, transformRotation);

            m_featureCalculator->SetLandmarks(l);

            cv::Mat features = m_featureCalculator->GetFeatures(cloud);
            cv::Mat unrolled = features.reshape(0, 120);
            unrolled.convertTo(unrolled, CV_64FC1);
            cv::Mat frame = fer::RV_preprocessDepthFrame(unrolled);

            std::vector<double> bgPerc(m_ferParameters.noPatches);
            bgPerc.assign(m_ferParameters.noPatches, 0);

            std::vector<cv::Mat> featData;
            fer::RV_featExtractionSingleFrame(frame, m_ferParameters, &bgPerc, featData);

            std::vector<double> weights(featData.size());
            weights.assign(featData.size(), 1.0 / featData.size());
            std::vector<double> finalProbs(m_numberOfClasses);
            finalProbs.assign(m_numberOfClasses, 0);

            for (int i = 0; i < featData.size(); i++)
            {
                cv::Mat localData = featData[i];
                std::vector<fer::randomTree> localForest = m_forests[i];
                if (bgPerc[i] < 0.5)
                {
                    std::vector<double> probs = RV_testForest(localData, localForest, m_ferParameters, m_numberOfClasses);
                    for (int k = 0; k < m_numberOfClasses; k++)
                    {
                        finalProbs[k] += probs[k] * weights[i];
                    }
                }
            }

            m_ferResult = finalProbs;
            m_dataReady = true;
            m_workerBusy = false;
        }
    }

}