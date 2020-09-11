#include "PointCloudIO.h"
#include "dbscan.h"
#include <pcl/kdtree/kdtree_flann.h>

int DBSCAN::run()
{
	int cID = 1;
	printf("Cluster: %d \n", cID);

	for (std::size_t i = 0; i < m_num_pts; ++i) //iterate through point list
	{
		if (m_fullpoints[i].clusterID == UNCLASSIFIED) //analyze unclassified points
		{
			if (expandCluster(i, cID) != FAILURE) //if point is a core point, expand the cluster and analyze its nbrs
			{
				cID += 1;						//Expand cluster comleted. Increase cID for next cluster.
				printf("Cluster: %d \n", cID);
			}
		}
	}

	return 0;
}

int DBSCAN::expandCluster(std::size_t i, int cID)
{ 
	std::vector<int> pointIDRadiusSearch = getNeighbors(m_cloudptr->points[i]); 

	if(pointIDRadiusSearch.size() < m_min_pts)
	{
		m_fullpoints[i].clusterID = NOISE;
		return FAILURE;
	}
	
	else  //PointIDRadiusSearch is queue of cluster points to analyze
	{
		for (std::vector<int>::iterator iter = pointIDRadiusSearch.begin(); iter != pointIDRadiusSearch.end(); ++iter)
		{
			m_fullpoints.at(*iter).clusterID = cID; //Set cluster ID (includes query point, at *iter =  0)
		}
	
	//Now we iterate through the Neighbor points --> This is our 'analysis queue'. We'll call these points 'queue points'.
		//--> We test if the 'queue point' is a dbscan core point (i.e getNeighbors > min_pts-1)
			//--> If yes, neighbors to the queue point are listed.
			//--> Points which have been given the cluster ID would have already been added to the 'analysis queue'.
			//--> Therefore, unclassified points are added to our 'analysis queue'.
	// --> The 'queue point' neighbours which are unclassified and noise points are then given the Cluster ID.

		pointIDRadiusSearch.erase(pointIDRadiusSearch.begin()); //remove query point from our list of points to analyze

		//pointIDRadiusSearch queue will increase with more pts to analyze, with push_back() (line 73)
		//--> push_back() invalidates all iterators on reallocation and invalidates the past-the-end iterator
		//--> define j and n. --> "j < n" makes our end_expr extendable.
		for (std::size_t j = 0, n = pointIDRadiusSearch.size(); j < n; ++j)
		{									 
			std::vector<int> nbrIDRadiusSearch = getNeighbors(m_cloudptr->points.at(pointIDRadiusSearch[j]));

			//If queue point is not a dbscan core point, its neighbors do not get added to cluster; keep iterating
			if (nbrIDRadiusSearch.size() < m_min_pts) 
			{
				continue;
			}

			else
			{
				nbrIDRadiusSearch.erase(nbrIDRadiusSearch.begin()); // remove queue point from our list of nbrs to analyze
																	// should already have the cluster ID (line 39 or 80)
				
				//Find unclassified neighboring points that are not already in the queue, and add them to it.
				for (std::vector<int>::iterator nbriter = nbrIDRadiusSearch.begin(); 
						nbriter != nbrIDRadiusSearch.end(); ++nbriter)
				{
					if (m_fullpoints.at(*nbriter).clusterID == UNCLASSIFIED) //&& 
					//std::find(pointIDRadiusSearch.begin(), pointIDRadiusSearch.end(),*nbriter) != pointIDRadiusSearch.end())
					{
						pointIDRadiusSearch.push_back(*nbriter); 
						n = pointIDRadiusSearch.size();
					}
				}

				for (std::vector<int>::iterator nbriter = nbrIDRadiusSearch.begin(); 
						nbriter != nbrIDRadiusSearch.end(); ++nbriter)
				{
					if (m_fullpoints.at(*nbriter).clusterID < 0)
					{
						m_fullpoints.at(*nbriter).clusterID = cID; //Both noise and unclassified given clusterID
					}
				}
			}
		}
		return SUCCESS;
	}
}

//Return array of indices, corresponding to  the neighborhood search.
//**Search's first return will be the query point!
std::vector<int> DBSCAN::getNeighbors(pcl::PointXYZ& pquery)
{
	std::vector<int> IDRadiusSearch; 
	std::vector<float> DistanceRadiusSearch;

	//std::cout << "Computing distance on point: " << pquery.x << " " << pquery.y << " " << pquery.z << std::endl;
	m_kdtree.radiusSearch(pquery, m_searchradius, IDRadiusSearch, DistanceRadiusSearch);
	
	return IDRadiusSearch;
}
