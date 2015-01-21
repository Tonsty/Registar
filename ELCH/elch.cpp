/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/elch.h>

#include <iostream>
#include <string>

#include <vector>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;
typedef std::pair<std::string, CloudPtr> CloudPair;
typedef std::vector<CloudPair> CloudVector;
typedef pcl::PolygonMeshPtr PolygonMeshPtr;
typedef std::pair<std::string, PolygonMeshPtr> PolygonMeshPair;
typedef std::vector<PolygonMeshPair> PolygonMeshVector;

bool
loopDetection (int end, const CloudVector &clouds, double dist, int &first, int &last)
{
  static double min_dist = -1;
  int state = 0;

  for (int i = end-1; i > 0; i--)
  {
    Eigen::Vector4f cstart, cend;
    //TODO use pose of scan
    pcl::compute3DCentroid (*(clouds[i].second), cstart);
    pcl::compute3DCentroid (*(clouds[end].second), cend);
    Eigen::Vector4f diff = cend - cstart;

    double norm = diff.norm ();

    //std::cout << "distance between " << i << " and " << end << " is " << norm << " state is " << state << std::endl;

    if (state == 0 && norm > dist)
    {
      state = 1;
      //std::cout << "state 1" << std::endl;
    }
    if (state > 0 && norm < dist)
    {
      state = 2;
      //std::cout << "loop detected between scan " << i << " (" << clouds[i].first << ") and scan " << end << " (" << clouds[end].first << ")" << std::endl;
      if (min_dist < 0 || norm < min_dist)
      {
        min_dist = norm;
        first = i;
        last = end;
      }
    }
  }
  //std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
  if (min_dist > 0 && (state < 2 || end == int (clouds.size ()) - 1)) //TODO
  {
    min_dist = -1;
    return true;
  }
  return false;
}

int
main (int argc, char **argv)
{
  double small_dist = 0.1;
  pcl::console::parse_argument (argc, argv, "--small_dist", small_dist);

  double large_dist = 3.0;
  pcl::console::parse_argument (argc, argv, "--large_dist", large_dist);

  double ran_thresh = 0.1;
  pcl::console::parse_argument (argc, argv, "--ran_thresh", ran_thresh);

  int iter = 100;
  pcl::console::parse_argument (argc, argv, "--iter", iter);

  std::string directory = ".";
  pcl::console::parse_argument (argc, argv, "--directory", directory);

  pcl::registration::ELCH<PointType> elch;
  pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp (new pcl::IterativeClosestPoint<PointType, PointType>);
  icp->setMaximumIterations (iter);
  icp->setMaxCorrespondenceDistance (small_dist);
  icp->setRANSACOutlierRejectionThreshold (ran_thresh);
  elch.setReg (icp);

  std::vector<int> ply_indices;
  ply_indices = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

  CloudVector clouds;
  PolygonMeshVector polygonMeshs;
  pcl::PLYReader plyReader;
  for (size_t i = 0; i < ply_indices.size (); i++)
  {
    CloudPtr pc (new Cloud);
	pcl::PolygonMeshPtr polygonMesh(new pcl::PolygonMesh);
	plyReader.read(argv[ply_indices[i]], *polygonMesh);
	pcl::fromPCLPointCloud2(polygonMesh->cloud, *pc);
    clouds.push_back (CloudPair (argv[ply_indices[i]], pc));
	polygonMeshs.push_back (PolygonMeshPair (argv[ply_indices[i]], polygonMesh));
    std::cout << "loading file: " << argv[ply_indices[i]] << " size: " << pc->size () << std::endl;
    elch.addPointCloud (clouds[i].second);
  }

  int first = 0, last = 0;

  for (size_t i = 0; i < clouds.size (); i++)
  {

    if (loopDetection (int (i), clouds, large_dist, first, last))
    {
      std::cout << "Loop between " << first << " (" << clouds[first].first << ") and " << last << " (" << clouds[last].first << ")" << std::endl;
      elch.setLoopStart (first);
      elch.setLoopEnd (last);
      elch.compute ();
    }
  }

  for (size_t i = 0; i < clouds.size (); i++)
  {
    std::string result_filename (clouds[i].first);
    result_filename = directory + "/" + result_filename.substr (result_filename.rfind ("/") + 1);
	
	pcl::toPCLPointCloud2(*clouds[i].second, polygonMeshs[i].second->cloud);
	pcl::io::savePLYFile (result_filename.c_str (), *polygonMeshs[i].second);
    std::cout << "saving result to " << result_filename << std::endl;
  }

  return 0;
}
