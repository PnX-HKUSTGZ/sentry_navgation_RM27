// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "global_relocalization/core.hpp"
#include <algorithm>
#include <map>
#include <tuple>
namespace global_relocalization {
// Sort both voxel keys and point coordinates: input permutations and OMP scheduling
// cannot change centroid accumulation or nearest-neighbour tie order.
inline Cloud::Ptr deterministicVoxel(const Cloud & input,double leaf) {
  using Key=std::tuple<int64_t,int64_t,int64_t>;
  std::map<Key,std::vector<Eigen::Vector3d>> cells;
  for(const auto & p:input)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)) {
    Eigen::Vector3d v(p.x,p.y,p.z);
    cells[{int64_t(std::floor(v.x()/leaf)),int64_t(std::floor(v.y()/leaf)),int64_t(std::floor(v.z()/leaf))}].push_back(v);
  }
  auto out=std::make_shared<Cloud>();out->reserve(cells.size());
  for(auto & cell:cells){auto & points=cell.second;std::sort(points.begin(),points.end(),[](const auto&a,const auto&b){return std::make_tuple(a.x(),a.y(),a.z())<std::make_tuple(b.x(),b.y(),b.z());});Eigen::Vector3d sum=Eigen::Vector3d::Zero();for(const auto & v:points)sum+=v;sum/=points.size();out->emplace_back(sum.x(),sum.y(),sum.z());}
  return out;
}
}
