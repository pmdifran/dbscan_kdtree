//Class definitions and functon declarations for importing custom .txt pointcloud data, and converting it to a pcl type.
	//Structure of input data is: x,y,z,M3C2distance,Nx,Ny,Nz

#pragma once

#include <vector>
#include <string>
#include <iostream>	//for std::cout
#include <fstream>
#include <iomanip>  //for setprecision()
#include <stdlib.h> //for calloc()
#include <locale> //for csv reader

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

//Custom input data type xyz with numerous fields (M3C2 Distance, Normals, Cluster ID). 
typedef struct fullPoint_
{
	float x, y, z;  // X, Y, Z position
	float M3C2, nx, ny, nz;
	int clusterID; // clustered ID

	friend std::istream& operator >> (std::istream& in, fullPoint_& fp) //Stream extractor overload
	{
		in >> fp.x >> fp.y >> fp.z >> fp.M3C2 >> fp.nx >> fp.ny >> fp.nz;
		return in;
	}

	friend std::ostream& operator << (std::ostream& out, fullPoint_& fp) //Stream insersion overload
	{
		out << std::fixed << std::setprecision(8) << fp.x << " " << fp.y << " " << fp.z << 
			" " << fp.M3C2 << " " << fp.nx << " " << fp.ny << " " << fp.nz << " " << fp.clusterID << std::endl;
		return out;
	}

}fullPoint;

//Locale to treat commas, spaces, and newlines as whitespace. 
struct csv_reader : std::ctype<char> 
{
	csv_reader() 
		: std::ctype<char>(get_table()) {} //construct csv_reader by calling get_table

	static std::ctype_base::mask const* get_table() 
	{
		static std::vector<std::ctype_base::mask> rc(table_size, std::ctype_base::mask());

		//define these characters as whitespace
		rc[','] = std::ctype_base::space;
		rc['\n'] = std::ctype_base::space;
		rc[' '] = std::ctype_base::space;
		return &rc[0]; //returns ctype mapping table address.
	}
};

void readData(std::string filename, std::vector<fullPoint>& fullpoints); 

void writeFullFile(std::vector<fullPoint> fullpoints, std::string filename, int num_points);

void convert_toPCL(std::vector<fullPoint>& fullpoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr, unsigned int& num_points);

