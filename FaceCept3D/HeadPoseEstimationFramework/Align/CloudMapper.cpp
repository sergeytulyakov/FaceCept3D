#include "CloudMapper.h"

#include <pcl/common/common.h>

#include <pcl/registration/registration.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>

#include <pcl/correspondence.h>
#include <pcl/search/kdtree.h>

#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/thread_time.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "UI/ShowCloud.h"
#include "Exception/HPEException.h"
#include "Filter/Filters.h"
#include "TransformationEstimationHPE.h"

#include <pcl/io/pcd_io.h>

namespace hpe
{
    float CalculateDistance(pcl::PointXYZRGBA &p1, pcl::PointXYZRGBA &p2)
    {
        Eigen::VectorXf d(3);
        d(0) = p1.x - p2.x;
        d(1) = p1.y - p2.y;
        d(2) = p1.z - p2.z;

        return d.norm();
    }

    CloudMapper::CloudMapper()
        : m_icpParams(CloudMapper::ICPParams(0.05, 0.05, 0.000001, 100000, 100)),
          m_useDynamicICP(false), m_usePointToPlaneMetric(false), m_useNormalShooting(false),
          m_estimateScale(false)

    {

    }

    void CloudMapper::AddCloud(CloudMapper::Cloud::Ptr &cloud, CloudMapper::Landmarks &landmarks)
    {
        m_pointClouds.push_back(cloud);
        m_landmarks.push_back(landmarks);
    }

    CloudMapper::Clouds CloudMapper::GetAlignedClouds(TransformationsPtr transformations)
    {
        return AlignAll(m_pointClouds, m_landmarks);
    }

    CloudMapper::Cloud::Ptr CloudMapper::GetMergedCloud()
    {
        CloudMapper::Clouds alignedClouds = AlignAll(m_pointClouds, m_landmarks);
        CloudMapper::Cloud::Ptr merged(new CloudMapper::Cloud);
        auto it = alignedClouds.begin();
        do
        {
            auto current = (*it);
            *merged += *current;
            ++it;
        }
        while (it != alignedClouds.end());

        return merged;
    }

    CloudMapper::TransformationsPtr CloudMapper::GetTransforms()
    {
        CloudMapper::TransformationsPtr transforms(new CloudMapper::Transformations);

        AlignAll(m_pointClouds, m_landmarks, transforms);

        return transforms;
    }

    void CloudMapper::SetICPParams(CloudMapper::ICPParams params)
    {
        m_icpParams = params;
    }

    void CloudMapper::SetDynamicICPParams(CloudMapper::DynamicICPParams &params)
    {
        m_dynamicICPParams = params;
        SetUseDynamicICP(true);
    }

    void CloudMapper::SetUseDynamicICP(bool use)
    {
        m_useDynamicICP = use;
    }

    void CloudMapper::SetUseNormalShooting(bool use)
    {
        m_useNormalShooting = use;
    }

    void CloudMapper::SetEstimateScale(bool estimateScale)
    {
        m_estimateScale = estimateScale;
    }

    void CloudMapper::SetUsePointToPlaneMetric(bool use)
    {
        m_usePointToPlaneMetric = use;
    }

    Eigen::Matrix4f CloudMapper::GetTransformForTwoCloudsPointToPlane(CloudMapper::Cloud::Ptr &source,
            CloudMapper::Cloud::Ptr &target,
            ICPParams params)
    {
        CloudMapper::NormalCloud::Ptr normalSource(new CloudMapper::NormalCloud);
        pcl::copyPointCloud(*source, *normalSource);

        CloudMapper::NormalCloud::Ptr normalTarget(new CloudMapper::NormalCloud);
        pcl::copyPointCloud(*target, *normalTarget);

        typedef pcl::search::KdTree<CloudMapper::PointNormal> Search2;
        Search2::Ptr search(new Search2);

        pcl::NormalEstimation<CloudMapper::PointNormal, CloudMapper::PointNormal> estimation;
        estimation.setSearchMethod(search);
        estimation.setKSearch(10);
        estimation.setInputCloud(normalTarget);
        estimation.compute(*normalTarget);

        typedef pcl::registration::CorrespondenceRejectorSampleConsensus<CloudMapper::PointNormal> Rejector;

        pcl::Registration<CloudMapper::PointNormal, CloudMapper::PointNormal>::Ptr registration(new pcl::IterativeClosestPoint<CloudMapper::PointNormal, CloudMapper::PointNormal>);

        pcl::registration::CorrespondenceRejectorSampleConsensus<CloudMapper::PointNormal>::Ptr rejector(new pcl::registration::CorrespondenceRejectorSampleConsensus<CloudMapper::PointNormal>);
        rejector->setInlierThreshold(params.RansacRejectionThreshold);
        rejector->setMaximumIterations(params.MaxRANSACIterations);
        rejector->setInputSource(normalSource);
        rejector->setInputTarget(normalTarget);
        registration->setInputSource(normalSource);
        registration->setInputTarget(normalTarget);
        registration->setMaxCorrespondenceDistance(params.MaxCorrespondenceDistance);
        registration->setRANSACOutlierRejectionThreshold(params.RansacRejectionThreshold);
        registration->setTransformationEpsilon(params.TransformationEpsilon);
        registration->setMaximumIterations(params.MaxICPIterations);
        registration->setRANSACIterations(params.MaxRANSACIterations);
        registration->addCorrespondenceRejector(rejector);

        if (m_useNormalShooting)
        {
            typedef pcl::registration::CorrespondenceEstimationNormalShooting<CloudMapper::PointNormal, CloudMapper::PointNormal, CloudMapper::PointNormal> NormalShooting;
            NormalShooting::Ptr ce(new NormalShooting);
            registration->setCorrespondenceEstimation(ce);
        }

        typedef TransformationEstimationHPE<CloudMapper::PointNormal, CloudMapper::PointNormal> Custom;
        Custom::Ptr metric(new Custom);
        metric->SetWeights(m_weights);
        registration->setTransformationEstimation(metric);

        CloudMapper::NormalCloud::Ptr c(new CloudMapper::NormalCloud);
        registration->align(*c);
        //std::cout << "Fitness Score " << registration->getFitnessScore() << std::endl;

        return registration->getFinalTransformation();
    }

    Eigen::Matrix4f CloudMapper::GetTransformForTwoCloudsPointToPoint(CloudMapper::Cloud::Ptr &source,
            CloudMapper::Cloud::Ptr &target,
            ICPParams params)
    {
        pcl::Registration<CloudMapper::PointType, CloudMapper::PointType>::Ptr registration(new pcl::IterativeClosestPoint<CloudMapper::PointType, CloudMapper::PointType>);
        typedef pcl::registration::CorrespondenceRejectorSampleConsensus<CloudMapper::PointType> Rejector;

        Rejector::Ptr rejector(new Rejector);
        rejector->setInlierThreshold(params.RansacRejectionThreshold);
        rejector->setMaximumIterations(params.MaxRANSACIterations);
        rejector->setInputSource(source);
        rejector->setInputTarget(target);

        registration->setInputSource(source);
        registration->setInputTarget(target);
        registration->setMaxCorrespondenceDistance(params.MaxCorrespondenceDistance);
        registration->setRANSACOutlierRejectionThreshold(params.RansacRejectionThreshold);
        registration->setTransformationEpsilon(params.TransformationEpsilon);
        registration->setMaximumIterations(params.MaxICPIterations);
        registration->setRANSACIterations(params.MaxRANSACIterations);
        registration->addCorrespondenceRejector(rejector);

        CloudMapper::Cloud::Ptr c(new CloudMapper::Cloud);
        registration->align(*c);

        std::cout << "Fitness Score " << registration->getFitnessScore() << std::endl;

        return registration->getFinalTransformation();
    }

    Eigen::Matrix4f CloudMapper::GetTransformForTwoCloudsDynamically(CloudMapper::Cloud::Ptr &source,
            CloudMapper::Cloud::Ptr &target,
            CloudMapper::DynamicICPParams &dynamicParams)
    {

        int icpRound = 1;

        CloudMapper::Cloud::Ptr result(new Cloud);
        pcl::copyPointCloud(*source, *result);
        Eigen::Matrix4f finalTransformationMatrix = Eigen::Matrix4f::Identity();
        for (auto it = dynamicParams.begin(); it != dynamicParams.end(); ++it)
        {
            auto params = *it;
            Eigen::Matrix4f stepTransform = Eigen::Matrix4f::Identity();

            if (m_usePointToPlaneMetric)
            {
                stepTransform = GetTransformForTwoCloudsPointToPlane(result, target, params);
            }
            else
            {
                stepTransform = GetTransformForTwoCloudsPointToPoint(result, target, params);
            }

            pcl::transformPointCloud(*result, *result, stepTransform);

            finalTransformationMatrix = finalTransformationMatrix * stepTransform;
        }

        return finalTransformationMatrix;
    }

    Eigen::Matrix4f CloudMapper::GetTransformHavingLandmarks(CloudMapper::Cloud::Ptr &source, Common<CloudMapper::PointType>::Landmarks &l1,
            CloudMapper::Cloud::Ptr &target, Common<CloudMapper::PointType>::Landmarks &l2)
    {
        pcl::CorrespondencesPtr correspondences = LandmarksToCorresponencies(source, l1, target, l2);
        return GetTransformHavingCorrespondences(source, target, correspondences);
    }

    Eigen::Matrix4f CloudMapper::GetTransformHavingCorrespondences(CloudMapper::Cloud::Ptr &source, CloudMapper::Cloud::Ptr &target, pcl::CorrespondencesPtr &correspondences)
    {
        Eigen::Matrix4f initialAlignmentMatrix = ComputeInitialAlignment(source, target, correspondences);
        bool initialAlignmentSucceed = IsNotNAN(initialAlignmentMatrix);

        CloudMapper::Cloud::Ptr sourceTransformed(new CloudMapper::Cloud);
        if (initialAlignmentSucceed)
        {
            pcl::transformPointCloud(*source, *sourceTransformed, initialAlignmentMatrix);
        }
        else
        {
            sourceTransformed = source;
        }

        if (m_estimateScale)
        {
            ShowTwoCloudsInDifferentColors<CloudMapper::PointType>(sourceTransformed, target, pcl::CorrespondencesPtr(), "After IA");
        }

        CloudMapper::DynamicICPParams dynamicParams = m_dynamicICPParams;
        if (m_useDynamicICP == false)
        {
            dynamicParams.push_back(m_icpParams);
        }

        CloudMapper::Cloud::Ptr result = sourceTransformed;
        Eigen::Matrix4f finalTransformationMatrix = GetTransformForTwoCloudsDynamically(sourceTransformed, target, dynamicParams);


        Eigen::Matrix4f totalTransform;
        if (initialAlignmentSucceed)
        {
            totalTransform = finalTransformationMatrix * initialAlignmentMatrix;
        }
        else
        {
            totalTransform = finalTransformationMatrix;
        }

        pcl::transformPointCloud(*source, *result, totalTransform);

        return totalTransform;
    }

    CloudMapper::Clouds CloudMapper::AlignAll(CloudMapper::Clouds &clouds, CloudMapper::LandmarksVector &landmarksVector, TransformationsPtr transformations)
    {
        TransformationsPtr allTransformations(new Transformations);
        if (transformations != nullptr)
        {
            allTransformations = transformations;
        }

        if (clouds.size() != landmarksVector.size())
        {
            throw HPEException("CloudMapper::AlignAll - clouds.size() != landmarksVector.size()");
        }

        std::vector<Eigen::Matrix4f> stepTransformations;

        pcl::PointCloud<CloudMapper::PointType>::Ptr sourceCloud = clouds[0];

        for (int cloudIndex = 1; cloudIndex < clouds.size() - 1; cloudIndex++)
        {
            int sourceIndex = cloudIndex;
            int targetIndex = sourceIndex + 1;

            pcl::PointCloud<CloudMapper::PointType>::Ptr targetCloud = clouds[targetIndex];

            auto totalTransform = GetTransformHavingLandmarks(sourceCloud, landmarksVector[sourceIndex],
                                  targetCloud, landmarksVector[targetIndex]);

            Eigen::Matrix3f rotationMatrix = totalTransform.block<3, 3>(0, 0);
            Eigen::Vector3f translationVector = totalTransform.block<3, 1>(0, 3);
            Eigen::Quaternionf quaternion(rotationMatrix);
            Eigen::Matrix3f rotation2(quaternion);

            pcl::transformPointCloud(*sourceCloud, *sourceCloud, totalTransform);
            pcl::PointCloud<CloudMapper::PointType>::Ptr newCloud(new pcl::PointCloud<CloudMapper::PointType>);
            *sourceCloud += *targetCloud;
            sourceCloud = Voxelize<CloudMapper::PointType>(sourceCloud, 0.006f);

            //ShowCloud<CloudMapper::PointType>(sourceCloud);

            if (IsNotNAN(totalTransform) == false)
            {
                int q = 42;
            }

            std::cout << cloudIndex << std::endl;

            stepTransformations.push_back(totalTransform);
        }

        ShowCloud<CloudMapper::PointType>(sourceCloud);

        CloudMapper::Clouds result;

        auto targetCloud = clouds[clouds.size() - 1];

        for (int cloudIndex = 0; cloudIndex < clouds.size() - 1; cloudIndex++)
        {
            Eigen::Matrix4f accumulatedMatrix = stepTransformations[cloudIndex];

            for (int matrixIndex = cloudIndex + 1; matrixIndex < stepTransformations.size(); matrixIndex++)
            {
                accumulatedMatrix = accumulatedMatrix * stepTransformations[matrixIndex];
            }

            CloudMapper::Cloud::Ptr transformed(new CloudMapper::Cloud);
            pcl::transformPointCloud(*clouds[cloudIndex], *transformed, accumulatedMatrix);
            allTransformations->push_back(accumulatedMatrix);

            result.push_back(transformed);
        }

        result.push_back(targetCloud);

        return result;
    }

    Eigen::Matrix4f CloudMapper::ComputeInitialAlignment(Cloud::Ptr &cloud1, Cloud::Ptr &cloud2, pcl::CorrespondencesPtr &correspondences)
    {
        if (correspondences->empty())
        {
            return Eigen::Matrix4f::Identity();
        }

        pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> sac;
        sac.setInlierThreshold(0.5);
        sac.setInputSource(cloud1);
        sac.setInputTarget(cloud2);
        sac.setInputCorrespondences(correspondences);
        pcl::Correspondences inliers;
        sac.getCorrespondences(inliers);

        Eigen::Matrix4f transformation;
        if (m_estimateScale)
        {
            pcl::registration::TransformationEstimationSVDScale<PointType, PointType> transformationEstimation;
            transformationEstimation.estimateRigidTransformation(*cloud1, *cloud2, inliers, transformation);
        }
        else
        {
            pcl::registration::TransformationEstimationSVD<PointType, PointType> transformationEstimation;
            transformationEstimation.estimateRigidTransformation(*cloud1, *cloud2, inliers, transformation);
        }


        if (transformation == Eigen::Matrix4f::Identity() || IsNotNAN(transformation) == false)
        {
            std::cout << "Identity or NAN" << std::endl;
        }

        return transformation;
    }

    Common<CloudMapper::PointType>::Landmark CloudMapper::FindClosestPoint(CloudMapper::Cloud::Ptr &cloud, CloudMapper::PointType &p)
    {
        pcl::search::KdTree<CloudMapper::PointType> search;
        search.setInputCloud(cloud);

        std::vector<int> indices;
        std::vector<float> distances;
        search.nearestKSearch(p, 1, indices, distances);

        Common<CloudMapper::PointType>::Landmark result;
        result.index = indices[0];
        result.point = cloud->at(indices[0]);

        return result;
    }

    pcl::CorrespondencesPtr CloudMapper::LandmarksToCorresponencies(Cloud::Ptr &cloud1, CloudMapper::Landmarks &landmarks1, Cloud::Ptr &cloud2, CloudMapper::Landmarks &landmarks2)
    {
        pcl::CorrespondencesPtr result(new pcl::Correspondences);

        int resultSize = std::min(landmarks1.size(), landmarks2.size());

        for (int i = 0; i < resultSize; i++)
        {
            auto sourcePoint1 = landmarks1.at(i).point;
            auto sourcePoint2 = landmarks2.at(i).point;

            auto p1IsNotNan = IsNotNAN(sourcePoint1);
            auto p2IsNotNan = IsNotNAN(sourcePoint2);
            if (p1IsNotNan && p2IsNotNan)
            {
                CloudMapper::Landmarks::value_type pt1 = FindClosestPoint(cloud1, sourcePoint1);
                CloudMapper::Landmarks::value_type pt2 = FindClosestPoint(cloud2, sourcePoint2);

                float distance = CalculateDistance(pt1.point, pt2.point);
                pcl::Correspondence correspondence(pt1.index, pt2.index, distance);

                result->push_back(correspondence);
            }
        }

        return result;
    }

    bool CloudMapper::IsNotNAN(const Eigen::Matrix4f &x)
    {
        return ((x.array() == x.array())).all();
    }

    bool CloudMapper::IsNotNAN(CloudMapper::PointType &point)
    {
        return (boost::math::isnan(point.x) || boost::math::isnan(point.y) || boost::math::isnan(point.z)) == false;
    }
}