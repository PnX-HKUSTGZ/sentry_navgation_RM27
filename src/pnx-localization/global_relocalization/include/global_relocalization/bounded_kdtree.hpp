// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT

// While the following KdTree code is written from scratch, it is heavily inspired by the nanoflann library.
// Thus, the following original license of nanoflann is included to be sure.

// https://github.com/jlblancoc/nanoflann/blob/master/include/nanoflann.hpp
/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
 * Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
 * Copyright 2011-2024  Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * THE BSD LICENSE
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
// Exact-radius adapter modifications: Copyright 2026 RM27 contributors.
#pragma once
#include <small_gicp/ann/kdtree.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace global_relocalization {
// Exact nearest neighbor within the unchanged rejector's inclusive squared
// radius. Reuses the original tree, projections, 4D distances and traversal
// order. It cannot request an approximate or epsilon-terminated search.
template<class PointCloud, class Projection = small_gicp::AxisAlignedProjection>
class BoundedKdTree {
public:
  using Tree = small_gicp::KdTree<PointCloud, Projection>;
  BoundedKdTree(const Tree & tree, double max_squared_distance)
  : tree_(tree.kdtree), max_squared_distance_(max_squared_distance) {
    if(!std::isfinite(max_squared_distance)||max_squared_distance<0)
      throw std::invalid_argument("bounded KD radius must be finite and nonnegative");
  }
  size_t nearest_neighbor_search(const Eigen::Vector4d & query, size_t * index, double * distance) const {
    *index=std::numeric_limits<size_t>::max();
    *distance=std::nextafter(max_squared_distance_,std::numeric_limits<double>::infinity());
    if(tree_.nodes.empty()||!query.allFinite())return 0;
    bool found=false;
    visit(query,tree_.root,*index,*distance,found);
    return found?1:0;
  }
private:
  void visit(const Eigen::Vector4d & query,small_gicp::NodeIndexType node_index,
    size_t & index,double & distance,bool & found) const {
    const auto & node=tree_.nodes[node_index];
    if(node.left==small_gicp::INVALID_NODE) {
      for(size_t i=node.node_type.lr.first;i<node.node_type.lr.last;++i) {
        const double squared=(small_gicp::traits::point(tree_.points,tree_.indices[i])-query).squaredNorm();
        // Preserve original strict comparison and first-visited tie selection.
        // Nonfinite points cannot create a valid GICP correspondence.
        if(std::isfinite(squared)&&squared<distance) {
          index=tree_.indices[i];distance=squared;found=true;
        }
      }
      return;
    }
    const double diff=node.node_type.sub.proj(query)-node.node_type.sub.thresh;
    const auto first=diff<0.0?node.left:node.right;
    const auto second=diff<0.0?node.right:node.left;
    visit(query,first,index,distance,found);
    if(distance>diff*diff)visit(query,second,index,distance,found);
  }
  const small_gicp::UnsafeKdTree<PointCloud,Projection> & tree_;
  double max_squared_distance_;
};
}
namespace small_gicp::traits {
template<class PointCloud,class Projection>
struct Traits<global_relocalization::BoundedKdTree<PointCloud,Projection>> {
  static size_t nearest_neighbor_search(const global_relocalization::BoundedKdTree<PointCloud,Projection> & tree,
    const Eigen::Vector4d & query,size_t * index,double * distance) {
    return tree.nearest_neighbor_search(query,index,distance);
  }
};
}
