#ifndef CLOUDMAPPER_H
#define CLOUDMAPPER_H

#include <memory>


#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>


//#include "Provider_Old/DataProviderOld.h"

#include <Common.h>

namespace hpe
{
    /**
     \class	CloudMapper
    
     \brief	Class that performs Iterative closest point alighment.
    
     */

    class CloudMapper
    {
        public:
            typedef pcl::PointXYZRGBA PointType;
            typedef pcl::PointCloud<PointType> Cloud;

            typedef pcl::PointNormal PointNormal;
            typedef pcl::PointCloud<PointNormal> NormalCloud;

            typedef Common<PointType>::Landmarks Landmarks;
            typedef std::vector<Landmarks> LandmarksVector;
            typedef std::vector<Cloud::Ptr> Clouds;

            typedef std::vector<Eigen::Matrix4f> Transformations;
            typedef std::shared_ptr<std::vector<Eigen::Matrix4f>> TransformationsPtr;

            struct ICPParams
            {
                ICPParams(float distance, float threshold, float epsilon, int maxICPIterations, int maxRansacIterations)
                    : MaxCorrespondenceDistance(distance),
                      RansacRejectionThreshold(threshold),
                      TransformationEpsilon(epsilon),
                      MaxICPIterations(maxICPIterations),
                      MaxRANSACIterations(maxRansacIterations)
                {

                }

                float MaxCorrespondenceDistance;
                float RansacRejectionThreshold;
                float TransformationEpsilon;
                int MaxICPIterations;
                int MaxRANSACIterations;
            };

            typedef std::vector<ICPParams> DynamicICPParams;

            CloudMapper();

            void AddCloud(Cloud::Ptr &cloud, Landmarks &landmarks);
            Clouds GetAlignedClouds(TransformationsPtr transformations = nullptr);
            Cloud::Ptr GetMergedCloud();
            TransformationsPtr GetTransforms();

            void SetICPParams(ICPParams params);
            void SetDynamicICPParams(DynamicICPParams &params);
            void SetUseDynamicICP(bool use);
            void SetUseNormalShooting(bool use);
            void SetEstimateScale(bool estimateScale);
            void SetUsePointToPlaneMetric(bool use);
            void SetWeights(std::vector<double> &weights) {
                m_weights = weights;
            }

            Eigen::Matrix4f GetTransformForTwoCloudsPointToPlane(Cloud::Ptr &source,
                    Cloud::Ptr &target,
                    ICPParams params = ICPParams(0.05, 0.05, 0.000001, 100000, 100));


            Eigen::Matrix4f GetTransformForTwoCloudsPointToPoint(Cloud::Ptr &source,
                    Cloud::Ptr &target,
                    ICPParams params = ICPParams(0.05, 0.05, 0.000001, 100000, 100));

            Eigen::Matrix4f GetTransformForTwoCloudsDynamically(Cloud::Ptr &source,
                    Cloud::Ptr &target,
                    DynamicICPParams &dynamicParams);

            Eigen::Matrix4f GetTransformHavingLandmarks(Cloud::Ptr &source,
                    Common<PointType>::Landmarks &l1,
                    Cloud::Ptr &target,
                    Common<PointType>::Landmarks &l2);

            Eigen::Matrix4f GetTransformHavingCorrespondences(Cloud::Ptr &source,
                    Cloud::Ptr &target,
                    pcl::CorrespondencesPtr &correspondences);


            pcl::CorrespondencesPtr LandmarksToCorresponencies(Cloud::Ptr &cloud1, Landmarks &landmarks1,
                    Cloud::Ptr &cloud2, Landmarks &landmarks2);

            Eigen::Matrix4f ComputeInitialAlignment(Cloud::Ptr &cloud1,
                                                    Cloud::Ptr &cloud2,
                                                    pcl::CorrespondencesPtr &correspondencies);



        private:
            Clouds m_pointClouds;
            LandmarksVector m_landmarks;

            Clouds AlignAll(Clouds &clouds, LandmarksVector &landmarksVector, TransformationsPtr transformations = nullptr);
            static Common<PointType>::Landmark FindClosestPoint(Cloud::Ptr &cloud, PointType &p);

            ICPParams m_icpParams;
            DynamicICPParams m_dynamicICPParams;
            bool m_useDynamicICP;
            bool m_usePointToPlaneMetric;
            bool m_useNormalShooting;
            bool m_estimateScale;

            std::vector<double> m_weights;

            static bool IsNotNAN(const Eigen::Matrix4f &x);
            static bool IsNotNAN(CloudMapper::PointType &point);
    };
}

#endif // CLOUDMAPPER_H
