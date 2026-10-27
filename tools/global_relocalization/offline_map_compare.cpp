// Frozen Core replay with the map as the only experimental variable.
// Unlike offline_query, map replacement is deliberate and recorded. No GT is read.
#include "global_relocalization/core.hpp"
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace global_relocalization;
template<class T> void read(const YAML::Node& y, const char* key, T& value) {
  if (y[key]) value=y[key].as<T>();
}
nlohmann::json candidate(const Candidate& c) {
  nlohmann::json matrix=nlohmann::json::array();
  for(int row=0;row<4;++row) matrix.push_back({c.map_from_reference.matrix()(row,0),c.map_from_reference.matrix()(row,1),c.map_from_reference.matrix()(row,2),c.map_from_reference.matrix()(row,3)});
  return {{"map_from_reference",matrix},{"fitness",c.fitness},{"inlier_ratio",c.inlier_ratio},{"converged",c.converged},{"within_search_side",c.within_search_side}};
}
int main(int argc,char** argv) {
  if(argc!=5) {std::cerr<<"usage: offline_map_compare MAP CAPTURE PROFILE OUTPUT\n";return 2;}
  try {
    const auto directory=std::filesystem::path(argv[2]);
    auto y=YAML::LoadFile(argv[3])["global_relocalization"]["ros__parameters"];
    if(!y) throw std::runtime_error("missing algorithm profile");
    Params p;
    read(y,"raw_deterministic",p.deterministic_preprocessing);
#define READ(key) read(y,#key,p.key)
    READ(candidate_workers);READ(bounded_global_correspondences);READ(search_side);READ(side_boundary_x);
    READ(sc_rings);READ(sc_sectors);READ(top_k);READ(fallback_candidate_count);READ(ring_key_candidates);READ(retrieval_all_views);
    READ(db_grid_resolution);READ(db_base_z);READ(db_base_z_offsets);READ(db_max_radius);READ(db_min_points);READ(db_min_elevation_deg);READ(db_max_elevation_deg);
    READ(gicp_leaf_size);READ(gicp_max_correspondence);READ(gicp_max_iterations);READ(max_fitness);READ(min_inlier_ratio);READ(ambiguity_margin);READ(confirmation_max_translation);READ(confirmation_max_rotation);
    READ(confirmation_gravity_prior);READ(confirmation_gravity_sigma);
#undef READ
    if(y["confirmation_map_up"]){const auto v=y["confirmation_map_up"].as<std::vector<double>>();if(v.size()!=3)throw std::invalid_argument("confirmation_map_up requires three entries");p.confirmation_map_up=Eigen::Vector3d(v[0],v[1],v[2]);}
    int count=3;read(y,"raw_confirmation_frames",count);
    if(count<3) throw std::runtime_error("at least three independent scans required");
    p.minimum_confirmation_scans=count;
    nlohmann::json metadata;std::ifstream(directory/"retrieval.json")>>metadata;
    const auto offset_json=metadata.at("sensor_offset");
    Eigen::Vector3d offset(offset_json[0].get<double>(),offset_json[1].get<double>(),offset_json[2].get<double>());
    Cloud query;
    if(pcl::io::loadPCDFile((directory/"query.pcd").string(),query)<0) throw std::runtime_error("cannot read frozen query");
    std::vector<Cloud> independent;
    for(int i=0;i<count;++i) {
      const auto path=directory/("independent_"+std::to_string(i)+".pcd");
      if(!std::filesystem::exists(path)) break;
      Cloud scan;if(pcl::io::loadPCDFile(path.string(),scan)<0) throw std::runtime_error("cannot read independent scan");
      independent.push_back(std::move(scan));
    }
    Core core(p);core.loadMap(argv[1]);
    auto prepared=core.prepare(query,offset,[]{return false;});
    auto result=prepared.diagnostics();
    const bool ready=prepared.ready();
    const bool confirm=ready && independent.size()==size_t(count);
    if(confirm) result=core.confirm(prepared,independent,[]{return false;});
    nlohmann::json output={{"ground_truth_used",false},{"prepared_ready",ready},{"confirmation_replayed",confirm},
      {"independent_scans_available",independent.size()},{"success",result.success},{"reason",result.reason},
      {"captured_map_sha256",metadata.at("map_sha256")},{"evaluated_map_sha256",core.mapHash()},
      {"candidate",candidate(result.candidate)},{"hypotheses",nlohmann::json::array()}};
    for(const auto& c:result.hypotheses) output["hypotheses"].push_back(candidate(c));
    std::ofstream stream(argv[4]);if(!stream) throw std::runtime_error("output unavailable");
    stream<<output.dump(2)<<'\n';
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
