// Functions for importing custom .txt pointcloud data, and converting it to a pcl type.

#include "dbscan.h" //includes our custom point cloud structures, and those from pcl.

//Read data and populate into custom structure 'fullPoint'
void readData(std::string filename, std::vector<fullPoint>& fullpoints)
{
	//get number of lines in file, for memory allocation.
	int count = 0;
	std::string line;
	std::ifstream in(filename);
	while (std::getline(in, line))
		count++;

	in.clear();
	in.seekg(0); //return to beginning of stream

	//test to see if the file has a header
	bool isheader;
	fullPoint testfp;
	std::getline(in, line);
	std::istringstream iss(line);

	if (iss >> testfp) //if header not detected, do not skip the first line. 
	{
		isheader = false;
		in.clear();
		in.seekg(0);
		std::cout << "Header Not Detected" << std::endl;
	}
	else 
	{
		isheader = true;
		std::cout << "Header Detected" << std::endl;
	}

	unsigned int num_points = count - isheader; //number of points -1 if a header was detected
	std::cout << "Number of points: " << num_points << std::endl;

	//Read the stream and import into fullpoints
	in.imbue(std::locale(std::locale(), new csv_reader())); //for the rest of stream, treat commas as whitespace
	fullPoint* fp = (fullPoint*)calloc(num_points, sizeof(fullPoint)); //initialize heap memory; point to first element
	while (in >> *fp) //Parse with overloaded stream extractor
	{
		fp->clusterID = UNCLASSIFIED;
		fullpoints.push_back(*fp);
		fp++;
	}

	in.close();
}

void writeFullFile(std::vector<fullPoint> fullpoints, std::string filename, int num_points)
{
	//Create appended output file name
	size_t lastindex = filename.find_last_of(".");
	std::string outfile = filename.substr(0, lastindex);
	outfile.append("_Clustered.txt");

	//Write data into file stream
	std::ofstream out(outfile);
	out << "//X Y Z M3C2_distance Nx Ny Nz ClusterID" << std::endl; // write header

	unsigned int i = 0;
	while (i < num_points)
	{ 
		out << fullpoints[i];
		i++;
	}
	out.close();
}

//Copy xyz coordinates from custom data struct 'fullPoint' to a PCL::Pointcloud<pcl::PointXYZ>::Ptr (smart, shared ptr)
void convert_toPCL(std::vector<fullPoint>& fullpoints, 
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr,unsigned int& num_points)
{
	//initialize point cloud shape. 
	cloudptr->width = num_points;
	cloudptr->height = 1;	//'height' is 1 for unorganized (i.e not pixelated) point clouds.
	cloudptr->points.resize(cloudptr->width * cloudptr->height);

	for (std::size_t i = 0; i < num_points; i++)
	{
		cloudptr->points[i].x = fullpoints[i].x;
		cloudptr->points[i].y = fullpoints[i].y;
		cloudptr->points[i].z = fullpoints[i].z;
	}
}