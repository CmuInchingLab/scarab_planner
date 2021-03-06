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
// using namespace nanoflann;

#include "KDTreeVectorOfVectorsAdaptor.h"

#include <iterator>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <fstream>


class KDTree{
    typedef std::vector<std::vector<double> > my_vector_of_vectors_t;
    typedef KDTreeVectorOfVectorsAdaptor< my_vector_of_vectors_t, double >  my_kd_tree_t;
    
    my_kd_tree_t *tree = NULL;
    my_vector_of_vectors_t point_cloud;
    size_t  dim = 3; 
    bool tree_built = false;

public:
    KDTree(){}
    KDTree(std::string fname){
        //read in a line of x,y,z values from .xyz file
        //https://stackoverflow.com/questions/7868936/read-file-line-by-line-using-ifstream-in-c
        std::ifstream infile(fname);
        if(!infile.fail()){
 
            double x, y, z;
            while (infile >> x >> y >> z){
                // process pair (x,y,z)
                std::vector<double> tuple{x,y,z};
                this->point_cloud.push_back(tuple);

                // print the numbers to stdout
                // std::cout << x <<" " <<  y << " "<< z << std::endl;
            }
            std::cout << "DONE READING " << fname << std::endl;	
            this->tree = new my_kd_tree_t(dim /*dim*/, this->point_cloud, 10 /* max leaf */ );
            this->tree->index->buildIndex();
            this->tree_built = true;
        }
        else{ //Error opening file
            std::cout << "ERROR IN kd_tree.hpp! TREE NOT BUILT. CHECK IF "<< fname <<" IS NAMED CORRECTLY" << std::endl;
        }

    }
    ~KDTree(){
        delete tree;
    }

    // inputs: a query point (x,y)
    // output: return z value at x,y or -1 (if tree wasn't construcuted)
    double query(double q_x, double q_y){
        if (this->tree_built){
            std::vector<double> query_pt = {q_x, q_y};
            const size_t num_results = 5;
            std::vector<size_t> ret_indexes(num_results);
            std::vector<double> out_dists_sqr(num_results);

            nanoflann::KNNResultSet<double> resultSet(num_results);

            resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
            //TODO: what does nanoflann::SearchParams(1) do???
            this->tree->index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(1)); 

            // // std::cout << "knnSearch(nn="<<num_results<<"): \n";
            // for (size_t i = 0; i < num_results; i++)
            //     std::cout << "ret_index["<<i<<"]=" << ret_indexes[i] << " out_dist_sqr=" << out_dists_sqr[i] << std::endl;
            // std::cout << "z val:" << samples[ret_indexes[0]][2] << std::endl;
            // std::cout << "index: " << ret_indexes[0] << std::endl;
            //std::cout << "KD pt: [" << q_x << ", " << q_y << ", " << this->point_cloud[ret_indexes[0]][2] << "]    .xyz index: " << ret_indexes[0] << std::endl;
            return this->point_cloud[ret_indexes[0]][2];    //return z val
        }
        return -1;
    }

};
