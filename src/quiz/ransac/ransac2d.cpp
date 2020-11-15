/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

struct Point3d
{
    float x;
    float y;
    float z;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData2D()
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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while (maxIterations--)
	{
        std::unordered_set<int> inliers;
        while (inliers.size() < 2)
        {
            inliers.insert(rand() % (cloud->points.size()));
        }


        float x1, y1, x2, y2;

        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        float a = (y1 - y2);
        float b = (x2 - x1);
        float c = (x1*y2) - (x2*y1);

        for(int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index)>0)
                continue;

            pcl::PointXYZ point = cloud->points[index];
            float x3 = point.x;
            float y3 = point.y;

            float d = fabs((a*x3) + (b*y3) + c)/ sqrt((a*a) + (b*b));

            if (d <= distanceTol)
                inliers.insert(index);


        }

        if (inliers.size()>inliersResult.size())
        {
            inliersResult = inliers;
        }
	}

    auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    int dimension = 3;
    Point3d p[dimension];

    float a, b, c, d;

    while (maxIterations--)
    {
        std::unordered_set<int> inliers;
        while (inliers.size() < dimension)
        {
            inliers.insert(rand() % (cloud->points.size()));
        }

        auto itr = inliers.begin();
        for(int i = 0; i < inliers.size();  i++)
        {
            p[i].x = cloud->points[*itr].x;
            p[i].y = cloud->points[*itr].y;
            p[i].z = cloud->points[*itr].z;
            itr++;
        }

        a = (p[1].y-p[0].y) * (p[2].z-p[0].z) - (p[1].z-p[0].z) * (p[2].y-p[0].y);
        b = (p[1].z-p[0].z) * (p[2].x-p[0].x) - (p[1].x-p[0].x) * (p[2].z-p[0].z);
        c = (p[1].x-p[0].x) * (p[2].y-p[0].y) - (p[1].y-p[0].y) * (p[2].x-p[0].x);
        d = -(a*p[0].x + b*p[0].y + c*p[0].z);

        for(int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index)>0)
                continue;

            pcl::PointXYZ point = cloud->points[index];
            float x = point.x;
            float y = point.y;
            float z = point.z;

            float distance = std::fabs((a*x) + (b*y) + (c*z) + d) / std::sqrt((a*a) + (b*b) + (c*c));

            if (distance <= distanceTol)
                inliers.insert(index);


        }

        if (inliers.size()>inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData2D();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
//	std::unordered_set<int> inliers = Ransac2D(cloud, 20, 0.5);
	std::unordered_set<int> inliers = Ransac3D(cloud, 20, 0.3);

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
