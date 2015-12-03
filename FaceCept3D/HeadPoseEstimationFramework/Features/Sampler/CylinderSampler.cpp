#define _CRT_SECURE_NO_WARNINGS

#include <opencv2\opencv.hpp>

#include "CylinderSampler.h"

#include <Exception/HPEException.h>
#include <pcl/common/common.h>

#include <boost/assign/std/vector.hpp>

#include <UI/ShowCloud.h>

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>

using namespace boost::assign;

namespace hpe
{
    CylinderSampler::CylinderSampler(Range phiRange, Range zRange, int topRows, int bottomRows, int sampleColumns)
        : m_phiRange(phiRange), m_zRange(zRange), m_topRows(topRows),
          m_bottomRows(bottomRows), m_visualize(false), m_sampleColumns(sampleColumns),
          m_supportOptimizedSampling(false)
    {
        //m_rightEyeIndices += 0, 4;
        //m_leftEyeIndices += 8, 12;
        //m_rightEyeIndices += 0;
        //m_rightEyeIndices += 0, 1, 2, 3, 4, 5, 6, 7;
        //m_leftEyeIndices += 1;
        //m_leftEyeIndices += 8, 9, 10, 11, 12, 13, 14, 15;
        m_leftEyeIndices += 0;
        m_rightEyeIndices += 1;
    }

    CylinderSampler::~CylinderSampler(void)
    {
    }

    pcl::PointCloud<CylinderSampler::PointType>::Ptr CylinderSampler::SampleCloud(pcl::PointCloud<PointType>::Ptr &cloud)
    {
        CloudType::Ptr res(new CloudType);
        return res;
    }

    pcl::PointCloud<CylinderSampler::PointType>::Ptr CylinderSampler::SampleCloud(pcl::PointCloud<PointType>::Ptr &cloud, const Common<PointType>::Landmarks &landmarks, pcl::ModelCoefficients &coefficients)
    {
        CloudType::Ptr res(new CloudType);

        PointType leftEye = MeanPoint(landmarks, m_leftEyeIndices);
        PointType rightEye = MeanPoint(landmarks, m_rightEyeIndices);

        Eigen::Vector3f betweenEyeVector = leftEye.getArray3fMap() - rightEye.getArray3fMap();
        float eyeDistance = betweenEyeVector.norm();

        pcl::ModelCoefficients cylinder = ComputeCylinder(rightEye, leftEye);

        Eigen::Vector3f direction;
        direction << 0, 0, 1; // Works only when the cloud is oriented along OZ
        Range phiRange(CV_PI * m_phiRange.first, CV_PI * m_phiRange.second);

        float eyeY = cylinder.values[1];

        float maxY = eyeY + m_zRange.second * eyeDistance;
        float minY = eyeY - m_zRange.first * eyeDistance;

        int totalRows = m_bottomRows + m_topRows;
        float stepSize = (maxY - minY) / totalRows;

        std::vector<float> samplingYs;
        float samplingYStart = eyeY + (m_topRows - 1) * stepSize;
        for (int i = 0; i < totalRows; i++)
        {
            samplingYs.push_back(samplingYStart);
            samplingYStart -= stepSize;
        }

        CloudType::Ptr samplingCylinderTop = GenerateSamplingCloud(cylinder, direction, phiRange, m_sampleColumns, samplingYs);

        coefficients.values.clear();

        if (m_supportOptimizedSampling)
        {
            coefficients.values.push_back(cylinder.values[0]);
            coefficients.values.push_back(cylinder.values[1]);
            coefficients.values.push_back(cylinder.values[2]);

            coefficients.values.push_back(phiRange.first);
            coefficients.values.push_back(phiRange.second);
            coefficients.values.push_back(m_sampleColumns);

            coefficients.values.push_back(eyeY + (m_topRows - 1) * stepSize);
            coefficients.values.push_back(eyeY - m_bottomRows * stepSize);
            coefficients.values.push_back(totalRows);
        }

        if (m_visualize)
        {
            std::cout << samplingCylinderTop->size() << std::endl;

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgbCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::copyPointCloud(*cloud, *rgbCloud);

            CloudType::Ptr eyes(new CloudType);
            eyes->push_back(leftEye);
            eyes->push_back(rightEye);

            std::vector<float> eyeYs;
            eyeYs.push_back(eyeY);
            auto eyeLine = GenerateSamplingCloud(cylinder, direction, phiRange, m_sampleColumns, eyeYs);

            std::vector<pcl::visualization::Camera> cameras;

            pcl::visualization::PCLVisualizer vEyes("Eyes");
            vEyes.addPointCloud(rgbCloud, "c");
            vEyes.addPointCloud<PointType>(eyes, "e");
            vEyes.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "e");
            vEyes.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "e");
            vEyes.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "c");
            vEyes.setBackgroundColor(1, 1, 1);

            vEyes.spin();

            vEyes.getCameras(cameras);

            pcl::visualization::PCLVisualizer v("");
            v.addPointCloud(rgbCloud, "c");
            v.addPointCloud<PointType>(samplingCylinderTop, "cyl");
            v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cyl");
            v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "cyl2");

            v.addCoordinateSystem();
            v.addPointCloud<PointType>(eyeLine, "EyeLine");
            v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "EyeLine");
            v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "EyeLine");
            v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "c");
            v.setBackgroundColor(1, 1, 1);

            v.setCameraParameters(cameras[0]);
            v.spin();

        }

        return samplingCylinderTop;
    }

    pcl::ModelCoefficients CylinderSampler::ComputeCylinder(PointType p1, PointType p2)
    {
        pcl::ModelCoefficients result;

        Eigen::Vector3f distanceVector = p1.getArray3fMap() - p2.getArray3fMap();
        float distance = distanceVector.norm();

        PointType middlePoint;
        middlePoint.getArray3fMap() = p1.getVector3fMap() - distanceVector / 2;

        float r = distance * 1.2;
        float rsq = r * r;
        float offset = std::sqrt(distance * distance - (distance / 2) * (distance / 2));

        float a = distance / 2;
        float hsq = rsq - a * a;

        Eigen::Vector3f P2 = p1.getArray3fMap() + a * (distance / 2);

        float x = P2[0] + sqrt(hsq) * (p2.y - p1.y) / distance;
        float y = P2[1] - sqrt(hsq) * (p2.x - p1.x) / distance;

        middlePoint.z -= offset;

        result.values.resize(7);
        result.values[0] = middlePoint.x;
        result.values[1] = middlePoint.y;
        result.values[2] = middlePoint.z;

        result.values[3] = 0;
        result.values[4] = 1;
        result.values[5] = 0;

        result.values[6] = r;

        return result;
    }

    CylinderSampler::PointType CylinderSampler::MeanPoint(const Common<PointType>::Landmarks &landmarks, const std::vector<int> &indices)
    {
        PointType res;
        Eigen::Vector3f result;
        result << 0, 0, 0;
        for (int i = 0; i < indices.size(); i++)
        {
            Eigen::Vector3f p = landmarks[indices[i]].point.getArray3fMap();
            result += p;
        }
        result /= indices.size();
        res.getArray3fMap() = result;
        return res;
    }

    CylinderSampler::CloudType::Ptr CylinderSampler::GenerateSamplingCloud(pcl::ModelCoefficients cylinder, Eigen::Vector3f direction, Range phiRange, int phiSteps, std::vector<float> samplingYs)
    {
        float radius = cylinder.values[6];
        float phi = phiRange.first;
        float phiStep = (phiRange.second - phiRange.first) / phiSteps;
        CloudType::Ptr result(new CloudType);
        for (int currentPhi = 0; currentPhi < phiSteps; ++currentPhi)
        {
            PointType p;
            p.x = radius * std::cos(phi);
            p.z = radius * std::sin(phi);

            Eigen::Vector3f normalVector;
            normalVector << p.x, p.y, p.z;
            normalVector /= normalVector.norm();

            p.normal_x = -normalVector[0];
            p.normal_y = -normalVector[1];
            p.normal_z = -normalVector[2];

            for (int currentY = 0; currentY < samplingYs.size(); ++currentY)
            {
                PointType p2 = p;
                p2.y = samplingYs[currentY];
                result->push_back(p2);
            }

            phi += phiStep;
        }

        Eigen::Quaternionf rotation;
        Eigen::Vector3f sourceVector;
        sourceVector << 0, 0, 1;
        rotation.setFromTwoVectors(sourceVector, direction);


        Eigen::Matrix4f transofm = Eigen::Matrix4f::Identity();
        Eigen::Vector3f shift;
        shift << cylinder.values[0], 0, cylinder.values[2];
        transofm.block<3, 1>(0, 3) = shift;
        transofm.block<3, 3>(0, 0) = rotation.toRotationMatrix();

        pcl::transformPointCloud(*result, *result, transofm);

        PointType p13;
        p13.getArray3fMap() << cylinder.values[0], 0, cylinder.values[2];

        return result;
    }

    Eigen::Vector3f CylinderSampler::GetDirection(pcl::PointCloud<PointType>::Ptr &cloud, const Common<PointType>::Landmarks &landmarks)
    {
        pcl::NormalEstimation<PointType, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

        ne.setRadiusSearch(0.3);

        ne.compute(*cloud_normals);

        pcl::Normal normal = cloud_normals->points[landmarks[m_noseTipIndex].index];

        Eigen::Vector3f direction;
        direction << normal.normal_x, normal.normal_y, normal.normal_z;
        return direction;
    }

    void CylinderSampler::SetVisualize(bool visualizeSampler)
    {
        m_visualize = visualizeSampler;
    }

    void CylinderSampler::SetEyeIndices(std::vector<int> &leftEyeIndices, std::vector<int> &rightEyeIndices)
    {
        m_leftEyeIndices = leftEyeIndices;
        m_rightEyeIndices = rightEyeIndices;
    }

    void CylinderSampler::SetSupportOptimizedSampling(bool support)
    {
        m_supportOptimizedSampling = support;
    }

}