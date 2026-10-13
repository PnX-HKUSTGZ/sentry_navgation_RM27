// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
// Replay only frozen estimator inputs. No ROS, control, TF or ground-truth input.
#include "global_relocalization/core.hpp"
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace global_relocalization;
namespace {
template<class T> void read(const YAML::Node & yaml,const char* key,T & value){if(yaml[key])value=yaml[key].as<T>();}
nlohmann::json candidateJson(const Candidate & c) {
  nlohmann::json matrix=nlohmann::json::array();for(int row=0;row<4;++row)matrix.push_back({c.map_from_reference.matrix()(row,0),c.map_from_reference.matrix()(row,1),c.map_from_reference.matrix()(row,2),c.map_from_reference.matrix()(row,3)});
  return {{"map_from_reference",matrix},{"fitness",c.fitness},{"inlier_ratio",c.inlier_ratio},{"converged",c.converged},{"within_search_side",c.within_search_side},
    {"descriptor_distance",c.descriptor_distance},{"ring_key_rank",c.ring_key_rank},{"scan_context_rank",c.scan_context_rank},{"from_scan_context",c.from_scan_context},{"from_ring_key",c.from_ring_key},
    {"seed_xyz",{c.seed_translation.x(),c.seed_translation.y(),c.seed_translation.z()}},{"seed_yaw",c.seed_yaw}};
}
}
int main(int argc,char** argv) {
  if(argc<5||argc>6){std::cerr<<"usage: global_relocalization_offline_query MAP.pcd CAPTURE_DIR PROFILE.yaml RESULT.json [baseline|covered]\n";return 2;}
  try {
    const auto began=std::chrono::steady_clock::now();const auto directory=std::filesystem::path(argv[2]);
    auto yaml=YAML::LoadFile(argv[3])["global_relocalization"]["ros__parameters"];
    if(!yaml)throw std::runtime_error("profile must contain global_relocalization.ros__parameters");
    Params p;p.ring_key_candidates=5;p.retrieval_all_views=true;
#define READ(key) read(yaml,#key,p.key)
    READ(search_side);READ(side_boundary_x);READ(sc_rings);READ(sc_sectors);READ(top_k);READ(ring_key_candidates);READ(retrieval_all_views);
    READ(db_grid_resolution);READ(db_base_z);READ(db_max_radius);READ(db_min_points);READ(db_min_elevation_deg);READ(db_max_elevation_deg);
    READ(gicp_leaf_size);READ(gicp_max_correspondence);READ(gicp_max_iterations);READ(max_fitness);READ(min_inlier_ratio);READ(ambiguity_margin);READ(confirmation_max_translation);READ(confirmation_max_rotation);
#undef READ
    if(argc==6){const std::string mode=argv[5];if(mode=="baseline"){p.ring_key_candidates=0;p.retrieval_all_views=false;}else if(mode!="covered")throw std::invalid_argument("mode must be baseline or covered");}
    nlohmann::json metadata;std::ifstream(directory/"retrieval.json")>>metadata;
    const auto offset_json=metadata.at("sensor_offset");Eigen::Vector3d offset(offset_json[0].get<double>(),offset_json[1].get<double>(),offset_json[2].get<double>());
    Cloud query;if(pcl::io::loadPCDFile((directory/"query.pcd").string(),query)<0)throw std::runtime_error("cannot load frozen query");
    int confirmation_count=3;read(yaml,"raw_confirmation_frames",confirmation_count);
    std::vector<Cloud> independent;
    for(int i=0;i<confirmation_count;++i){Cloud scan;if(pcl::io::loadPCDFile((directory/("independent_"+std::to_string(i)+".pcd")).string(),scan)<0)throw std::runtime_error("missing frozen independent scan");independent.push_back(std::move(scan));}
    Core core(p);core.loadMap(argv[1]);if(core.mapHash()!=metadata.at("map_sha256").get<std::string>())throw std::runtime_error("map hash differs from captured query metadata");
    const auto result=core.search(query,independent,offset,[]{return false;});
    nlohmann::json out={{"success",result.success},{"reason",result.reason},{"ground_truth_used",false},{"map_sha256",result.map_hash},{"top_k",p.top_k},{"ring_key_candidates",p.ring_key_candidates},{"retrieval_all_views",p.retrieval_all_views},
      {"candidate_limit",p.top_k+p.ring_key_candidates},{"database_size",result.database_size},{"retrieval_scored_views",result.retrieval_scored_views},{"retrieval_candidate_count",result.retrieval_candidate_count},{"retrieval_ring_added",result.retrieval_ring_added},
      {"candidates_tested",result.candidates_tested},{"retrieval_sec",result.retrieval_sec},{"gicp_sec",result.gicp_sec},{"confirmation_sec",result.confirmation_sec},
      {"total_wall_sec",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},{"candidate",candidateJson(result.candidate)},{"hypotheses",nlohmann::json::array()}};
    for(const auto & c:result.hypotheses)out["hypotheses"].push_back(candidateJson(c));
    std::ofstream output(argv[4]);if(!output)throw std::runtime_error("cannot open replay result");output<<out.dump(2)<<'\n';
    std::cout<<"success="<<result.success<<" candidates="<<result.candidates_tested<<" fitness="<<result.candidate.fitness<<" wall="<<out["total_wall_sec"]<<" reason="<<result.reason<<'\n';
    return result.success?0:1;
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
}
