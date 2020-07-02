/*
 *  Copyright (C) 2018, Tsz-Ho Kwok in Concordia University, Montreal
 *  All rights reserved.
 *  
 *   
 *  Redistribution and use in source and binary forms, with or without modification, 
 *  are permitted provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimer in the documentation 
 *	   and/or other materials provided with the distribution.
 *  3. Acknowledge the provided code by citing the paper in any publication using this code.
 *     T.-H. Kwok, "DNSS: Dual-Normal Space Sampling for 3D ICP Registration", IEEE TASE, 2018.
 *
 *   THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 *   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 *   IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 *   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 *   OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 *   OF SUCH DAMAGE.
 */

#include "DualNormalSpaceSampling.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkAutoInit.h>
#include<string>
#include<vector>
#include<cmath>
int main()
{

	double x, y, z; //point positions
	double nx, ny, nz; //point normals

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtar(new pcl::PointCloud<pcl::PointXYZ>);

	// pcl::PCLPointCloud2 cloud_blob;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("球面1.pcd", *cloud) == -1) {

		PCL_ERROR("Could not read file \n");
	}
	//* the data should be available in cloud

	// Normal estimation*
	//法向计算
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//为kdtree添加点云数据
	tree->setInputCloud(cloud);

	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	//点云法向计算时，需要搜索的近邻点大小
	n.setKSearch(20);
	//开始进行法向计算
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	//将点云数据与法向信息拼接
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	list<MeshNode> nodeList;

	//store points by MeshNode
	for (int i=0;i<cloud->points.size();i++) 
	{

		MeshNode node;
		node.v[0] = cloud->points[i].x; node.v[1] = cloud->points[i].y; node.v[2] = cloud->points[i].z;
		node.n[0] = normals->points[i].normal_x; node.n[1] = normals->points[i].normal_y; node.n[2] = normals->points[i].normal_z;
		node.index = i;
		nodeList.push_back(node);
	}

	DualNormalSpaceSampling dnss(nodeList);

	list<int> SampleIndex; //the output -- stored the index of the nodes (start from 0)

	int NumPts = 0; //to get the target number of sample points
	printf("Number of sampling points: "); scanf("%d", &NumPts);
	cloudtar->width = NumPts;
	cloudtar->height = 1;
	cloudtar->points.resize(cloudtar->width*cloudtar->height);
	//list<int> SampleIndex(40000);
	if (dnss.Run(SampleIndex, NumPts)) 
	{
		printf("Selected %d points in Dual Normal Space Sampling\n", SampleIndex.size());
		printf("the indices of selected points are: ");
		int i = 0;
		for (auto const &index : SampleIndex) {
			//printf(&index);
			printf("index ",index);
			cloudtar->points[i] = cloud->points[index];
				i++;
		}printf("\n");
	}
	else printf("ERROR: Cannot select points by Dual Normal Space Sampling1\n");

	pcl::io::savePCDFileASCII("球面1dnss26192.pcd", *cloudtar);
	return 0;
}