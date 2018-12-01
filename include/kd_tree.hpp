/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2016 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#include <nanoflann.hpp>
using namespace nanoflann;

#include "KDTreeVectorOfVectorsAdaptor.h"

#include <iterator>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <fstream>


class KDTree{
    KDTree(std::string fname){
        
    }
};
const int SAMPLES_DIM = 3;

typedef std::vector<std::vector<double> > my_vector_of_vectors_t;

// void generateRandomPointCloud(my_vector_of_vectors_t &samples, const size_t N, const size_t dim, const double max_range = 10.0)
// {
// 	std::cout << "Generating "<< N << " random points...";
// 	samples.resize(N);
// 	for (size_t i = 0; i < N; i++)
// 	{
// 		samples[i].resize(dim);
// 		for (size_t d = 0; d < dim; d++)
// 			samples[i][d] = max_range * (rand() % 1000) / (1000.0);
// 	}
// 	std::cout << "done\n";
// }

void kdtree_demo(/*const size_t nSamples*/ my_vector_of_vectors_t &samples, const size_t dim)
{
	// my_vector_of_vectors_t  samples;

	// const double max_range = 20;

	// Generate points:
	// generateRandomPointCloud(samples, nSamples,dim, max_range);

	// Query point:
	// std::vector<double> query_pt(dim);
	// for (size_t d = 0;d < dim; d++){
	// 	query_pt[d] = max_range * (rand() % 1000) / (1000.0);
	// 	std::cout << query_pt[d] << " ";
	// }
	// std::cout << std::endl;
	std::vector<double> query_pt{-138.05, 150.98};

	// construct a kd-tree index:
	// Dimensionality set at run-time (default: L2)
	// ------------------------------------------------------------
	typedef KDTreeVectorOfVectorsAdaptor< my_vector_of_vectors_t, double >  my_kd_tree_t;

	my_kd_tree_t   mat_index(dim /*dim*/, samples, 10 /* max leaf */ );
	mat_index.index->buildIndex();

	// do a knn search
	const size_t num_results = 1;
	std::vector<size_t>  ret_indexes(num_results);
	std::vector<double> out_dists_sqr(num_results);

	nanoflann::KNNResultSet<double> resultSet(num_results);

	resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
	mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(1)); //TODO: what does nanoflann::SearchParams(1) do???

	// std::cout << "knnSearch(nn="<<num_results<<"): \n";
	for (size_t i = 0; i < num_results; i++)
		std::cout << "ret_index["<<i<<"]=" << ret_indexes[i] << " out_dist_sqr=" << out_dists_sqr[i] << std::endl;
	std::cout << "z val:" << samples[ret_indexes[0]][2] << std::endl;
}

void readFile(my_vector_of_vectors_t &point_cloud, const std::string &fname){
	std::ifstream infile(fname);
	//https://stackoverflow.com/questions/7868936/read-file-line-by-line-using-ifstream-in-c
	//read in a line of x,y,z values from file
	double x, y, z;
	while (infile >> x >> y >> z){
		// process pair (x,y,z)
		std::vector<double> tuple{x,y,z};
		point_cloud.push_back(tuple);

		// print the numbers to stdout
		// std::cout << x <<" " <<  y << " "<< z << std::endl;
	}
	std::cout << "DONE READING " << fname << std::endl;
}	
int main()
{
	std::vector<std::vector<double>> point_cloud;
	readFile(point_cloud, "victoria_crater.xyz");
	// Randomize Seed
	// srand(static_cast<unsigned int>(time(nullptr)));
	kdtree_demo(point_cloud /* samples */, SAMPLES_DIM /* dim */);
}