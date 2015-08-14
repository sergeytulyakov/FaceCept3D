#pragma once

#ifndef SHOWMESH_H
#define SHOWMESH_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>

namespace hpe
{
    void ShowMesh(pcl::PolygonMesh::Ptr &mesh, std::string caption = "Mesh")
    {
        pcl::visualization::PCLVisualizer viewer(caption);

        viewer.addPolygonMesh(*mesh, "mesh");
        //viewer.resetCamera();
        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "mesh");
        while (viewer.wasStopped() == false)
        {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
}

#endif // SHOWMESH_H
