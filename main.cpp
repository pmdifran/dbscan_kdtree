#include "PointCloudIO.h"
#include "dbscan.h"

int main (int argc, char* argv[])
{
//Declare inputs for the program
	std::string filename;
	unsigned int min_points;
	double searchradius;
	

//Declare objects used for the program
	std::vector<fullPoint> fullpoints;  //Initialize the vector. Stays alive throught the entire scope of main().
										//Vector metadata stored on the stack; vector data is stored on the heap.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>); //smart pointer declaration to heap. 
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

//Usage: Expect 7 arguments; (1)dbscan.exe; (2)-i; (3)<filename>; (4)-m; (5)<min points>; (6)-d; (7)<min distance>
//OR (1)dbscan.exe (2) test

	if (argc != 7 && argc != 2) 
	{
		std::cerr << "Usage: " << argv[0] << "-i <filename> -m <min points> -d <min distance> " << std::endl;
		std::cerr << "Test Usage: dbscan test";
		return 1;	//exit program if the number of arguments is not met
	}

	if (argc == 2)
	{
		if (std::string(argv[1]) != "test")
		{
			std::cerr << "Usage: " << argv[0] << "-i <filename> -m <min points> -d <min distance> " << std::endl;
			std::cerr << "Test Usage: dbscan test";
		}
		std::cout << "Launching DBSCAN Test Dataset" << std::endl;
		filename = "test.txt";
		min_points = 12;
		searchradius = 0.3;
	}
	else
	{
		for (int i = 1; i < argc; i++)
		{
			if (std::string(argv[i]) == "-i") {
				filename = argv[i + 1];
			}
			else if (std::string(argv[i]) == "-m") {
				min_points = atoi(argv[i + 1]);
			}

			else if (std::string(argv[i]) == "-d") {
				searchradius = atof(argv[i + 1]);
			}
		}
	}

	//Import data and create pcl pointcloud.
	std::cout << "File: " << filename << std::endl;
	std::cout << "Importing Data..." << std::endl;
	readData(filename, fullpoints); //read file and store into fullpoints
	std::cout << "Data Read" << std::endl;

	unsigned int num_points = fullpoints.size();

	convert_toPCL(fullpoints, cloudptr,num_points); //create a pcl::pointcloud pointer object; give it all our xyz data
	std::cout << "Pcl pointcloud created" << std::endl;

	kdtree.setInputCloud(cloudptr); //create kdtree from cloud.
	std::cout << "KDtree built" << std::endl;

	// dbscan program constructor. dbscan object takes ownership of fullpoints.
	DBSCAN dbscan(fullpoints,cloudptr, kdtree, min_points, searchradius);

	std::cout << "Beginning DBSCAN" << std::endl;
	dbscan.run(); //dbscan main run function
	printf("DBSCAN COMPLETE \n");

	writeFullFile(dbscan.get_m_fullpoints(), filename, num_points);
	printf("File Written \n");

	printf("\n -------------------Finished------------------ \n");

	return 0;

}