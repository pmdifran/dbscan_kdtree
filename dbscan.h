#pragma once
#include "PointCloudIO.h"
#include <pcl\kdtree\kdtree_flann.h>
#include <math.h>

//For another time: Try writing a custom PCL point type, and remove need to copy. 

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

class DBSCAN
{
private:
	std::vector<fullPoint> m_fullpoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudptr;
	pcl::KdTreeFLANN<pcl::PointXYZ> m_kdtree;
	unsigned int m_min_pts;
	double m_searchradius;
	unsigned int m_num_pts;

public:		//constructor with member initialize list. Pointclouds are passed in by reference.
	DBSCAN(std::vector<fullPoint>& fullpoints, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudptr,
		pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, unsigned int min_pts, double searchradius)
		:m_fullpoints(std::move(fullpoints))
		, m_cloudptr(cloudptr)
		, m_kdtree(kdtree)
		, m_min_pts(min_pts)
		, m_searchradius(searchradius)
	{
		m_num_pts = cloudptr->width;
		std::cout << "Min # points: " << m_min_pts << std::endl;
		std::cout << "Search radius: " << m_searchradius << std::endl;
	}

	~DBSCAN() {}

	//Methods (defined in dbscan.cpp)
	int run();
	int expandCluster(std::size_t index, int cID);
	std::vector<int> getNeighbors(pcl::PointXYZ& pquery);
	std::vector<fullPoint> get_m_fullpoints() { return m_fullpoints; }

};



