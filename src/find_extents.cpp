//#include "find_extents.h"
#include <ros/ros.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_ros/transforms.h>

using namespace std;
using namespace pcl;

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;

std::vector<geometry_msgs::Point> find_extents(pcl::PointCloud<PointT> pcd)
{
	ROS_INFO("Inside find_extents");
	vector<double> vecx, vecy, vecz;
	//float minx, miny, minz, maxx, maxy, maxz;
	for (std::vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it1 = pcd.points.begin(); it1 != pcd.points.end(); ++it1) {
		//cout << "cluster " << j << ": x = " << it1->x << " y = " << it1->y << " z = " << it1->z << endl;
		vecx.push_back(it1->x);
		vecy.push_back(it1->y);
		vecz.push_back(it1->z);
	}

	//vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it_minx, it_miny, it_minz,
	//it_maxx, it_maxy, it_maxz;
	vector<double>::iterator it_minx, it_miny, it_minz, it_maxx, it_maxy, it_maxz;
	it_minx = min_element(vecx.begin(), vecx.end());
	it_miny = min_element(vecy.begin(), vecy.end());
	it_minz = min_element(vecz.begin(), vecz.end());
	it_maxx = max_element(vecx.begin(), vecx.end());
	it_maxy = max_element(vecy.begin(), vecy.end());
	it_maxz = max_element(vecz.begin(), vecz.end());

	//float dim1, dim2, dim3;
	PointT pt_minx, pt_miny, pt_minz, pt_maxx, pt_maxy, pt_maxz;
	vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it2 = pcd.points.begin();

	//TODO: we could use size_t indes= std::distance(vecx.begin(), it_minx); to get an index from the iterators
	// this would be safer as there might be multiple points with the same coordinates in some component of the vector
	// and it would also safe some time

	for (vector<double>::iterator pos = vecx.begin(); pos != vecx.end(); ++pos) {
		if (pos == it_minx){
			pt_minx = *it2;
		}
		if (pos == it_maxx){
			pt_maxx = *it2;
		}
		++it2;
	}

	it2 = pcd.points.begin();
	for (vector<double>::iterator pos = vecy.begin(); pos != vecy.end(); ++pos) {
		if (pos == it_miny){
			pt_miny = *it2;
		}
		if (pos == it_maxy){
			pt_maxy = *it2;
		}
		++it2;
	}

	it2 = pcd.points.begin();
	for (vector<double>::iterator pos = vecz.begin(); pos != vecz.end(); ++pos) {
		if (pos == it_minz){
			pt_minz = *it2;
		}
		if (pos == it_maxz){
			pt_maxz = *it2;
		}
		++it2;
	}

	geometry_msgs::Point vertices[6];
	vertices[0].x = pt_minx.x;
	vertices[0].y = pt_minx.y;
	vertices[0].z = pt_minx.z;
	vertices[1].x = pt_maxx.x;
	vertices[1].y = pt_maxx.y;
	vertices[1].z = pt_maxx.z;
	vertices[2].x = pt_miny.x;
	vertices[2].y = pt_miny.y;
	vertices[2].z = pt_miny.z;
	vertices[3].x = pt_maxy.x;
	vertices[3].y = pt_maxy.y;
	vertices[3].z = pt_maxy.z;
	vertices[4].x = pt_minz.x;
	vertices[4].y = pt_minz.y;
	vertices[4].z = pt_minz.z;
	vertices[5].x = pt_maxz.x;
	vertices[5].y = pt_maxz.y;
	vertices[5].z = pt_maxz.z;

	std::vector<geometry_msgs::Point> extent;
	for (int i =0; i<6; i++)
		extent.push_back(vertices[i]);

	return extent;
}
