/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <math.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::array<float,3> get_line_coef(float x1, float x2, float y1, float y2)
{
	std::array<float,3> coeff {0.0f, 0.0f, 0.0f};
	coeff[0] = y1 - y2;
	coeff[1] = x2 - x1;
	coeff[2] = x1 * y2 - x2 * y1;
	return coeff;
}

std::array<float,4> get_plane_coef(float x1, float x2, float x3, float y1, float y2, float y3, float z1, float z2, float z3)
{
	std::vector<float> v1 {x2-x1, y2-y1, z2-z1};
	std::vector<float> v2 {x3-x1, y3-y1, z3-z1};
	// cross product of v1 and v2  ----->  normal vector v3
	std::vector<float> v3 {(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1),
	                          (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1),
	                          (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1)};
	
	float A = v3[0]; // i
	float B = v3[1]; // j
	float C = v3[2]; // k
	float D = -1.0f * (A*x1 + B*y1 + C*z1);
	std::array<float,4> coeff {A, B, C, D};
	return coeff;
}

float distance_between_line_point(float x, float y, std::array<float,3> coeff)
{
	float A = coeff[0];
	float B = coeff[1];
	float C = coeff[2];
	return fabs(A * x + B * y + C) / sqrt(A*A + B*B);
}

float distance_between_plane_point(float x, float y, float z, std::array<float,4> coeff)
{
	float A = coeff[0];
	float B = coeff[1];
	float C = coeff[2];
	float D = coeff[3];
	return fabs(A * x + B * y + C * z + D) / sqrt(A*A + B*B + C*C);
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int max_num_inliers = 0;
	// For max iterations 
	while (maxIterations--) {
		int num_inliers = 0;
		std::unordered_set<int> inliers;  // set prevent inserting the same indice twice
		while(inliers.size() < 2)
			inliers.insert(rand() % (cloud->points.size())); // Insert indice between 0 and size of cloud points
		// Randomly sample subset and fit line
		auto itr = inliers.begin();
		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;

		for (int index = 0; index < cloud->points.size(); index++) {
			// if line points include the selected point, continue
			if (inliers.count(index) > 0)
				continue;
			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier
			if (distance_between_line_point(cloud->points[index].x, cloud->points[index].y, get_line_coef(x1, x2, y1, y2)) < distanceTol) {
				num_inliers++;
				inliers.insert(index);
			}
		}

		if (num_inliers > max_num_inliers){
			max_num_inliers = num_inliers;
			inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int max_num_inliers = 0;
	// For max iterations 
	while (maxIterations--) {
		int num_inliers = 0;
		std::unordered_set<int> inliers;  // set prevent inserting the same indice twice
		while(inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size())); // Insert indice between 0 and size of cloud points
		// Randomly sample subset and fit line
		auto itr = inliers.begin();
		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		float z1 = cloud->points[*itr].z;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
		float z2 = cloud->points[*itr].z;
		itr++;
		float x3 = cloud->points[*itr].x;
		float y3 = cloud->points[*itr].y;
		float z3 = cloud->points[*itr].z;

		for (int index = 0; index < cloud->points.size(); index++) {
			// if line points include the selected point, continue
			if (inliers.count(index) > 0)
				continue;
			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier
			if (distance_between_plane_point(cloud->points[index].x, cloud->points[index].y, cloud->points[index].z, get_plane_coef(x1, x2, x3, y1, y2, y3, z1, z2, z3)) < distanceTol) {
				num_inliers++;
				inliers.insert(index);
			}
		}

		if (num_inliers > max_num_inliers){
			max_num_inliers = num_inliers;
			inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function

	// std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	// 3D RANSAC
	std::unordered_set<int> inliers = Ransac3D(cloud, 30, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
