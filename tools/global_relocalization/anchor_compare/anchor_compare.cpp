// Frozen-core experiment: score both coordinate anchors only after the same
// independent confirmation gates. No GT, requested pose or simulated state is read.
#include "global_relocalization/core.hpp"
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace global_relocalization;
using Json=nlohmann::json;
template<class T> void read(const YAML::Node & y,const char * key,T & value){if(y[key])value=y[key].as<T>();}
Json candidateJson(const Candidate & c) {
  Json matrix=Json::array();for(int r=0;r<4;++r)matrix.push_back({c.map_from_reference.matrix()(r,0),c.map_from_reference.matrix()(r,1),c.map_from_reference.matrix()(r,2),c.map_from_reference.matrix()(r,3)});
  return {{"map_from_reference",matrix},{"fitness",c.fitness},{"inlier_ratio",c.inlier_ratio},{"converged",c.converged},{"within_search_side",c.within_search_side},{"scan_context_rank",c.scan_context_rank},{"database_location",c.database_location}};
}
int main(int argc,char ** argv) {
  if(argc!=5){std::cerr<<"usage: anchor_compare MAP PROFILE JOBS.json RESULTS.json\n";return 2;}
  try {
    auto y=YAML::LoadFile(argv[2])["global_relocalization"]["ros__parameters"];Params p;
    read(y,"raw_deterministic",p.deterministic_preprocessing);
#define READ(key) read(y,#key,p.key)
    READ(candidate_workers);READ(bounded_global_correspondences);READ(search_side);READ(side_boundary_x);
    READ(sc_rings);READ(sc_sectors);READ(top_k);READ(fallback_candidate_count);READ(ring_key_candidates);READ(retrieval_all_views);
    READ(db_grid_resolution);READ(db_base_z);READ(db_base_z_offsets);READ(db_max_radius);READ(db_min_points);READ(db_min_elevation_deg);READ(db_max_elevation_deg);
    READ(gicp_leaf_size);READ(gicp_max_correspondence);READ(gicp_max_iterations);READ(max_fitness);READ(min_inlier_ratio);READ(ambiguity_margin);READ(confirmation_max_translation);READ(confirmation_max_rotation);
    READ(confirmation_gravity_prior);READ(confirmation_gravity_sigma);
#undef READ
    if(y["confirmation_map_up"]){const auto v=y["confirmation_map_up"].as<std::vector<double>>();if(v.size()!=3)throw std::invalid_argument("confirmation_map_up requires three entries");p.confirmation_map_up=Eigen::Vector3d(v[0],v[1],v[2]);}
    int count=3;read(y,"raw_confirmation_frames",count);if(count<3)throw std::invalid_argument("three confirmations required");p.minimum_confirmation_scans=count;
    Json jobs;std::ifstream(argv[3])>>jobs;Core core(p);core.loadMap(argv[1]);
    Json output={{"ground_truth_used",false},{"map_sha256",core.mapHash()},
      {"confirmation_gravity_prior",{{"enabled",p.confirmation_gravity_prior},{"sigma_rad",p.confirmation_gravity_sigma},
       {"inverse_variance",1./(p.confirmation_gravity_sigma*p.confirmation_gravity_sigma)},
       {"reference_gravity_source","caller_declared_gravity_aligned_reference"},
       {"map_up",{p.confirmation_map_up.x(),p.confirmation_map_up.y(),p.confirmation_map_up.z()}}}},{"results",Json::array()}};
    for(const auto & job:jobs) {
      const auto start=std::chrono::steady_clock::now();const auto directory=std::filesystem::path(job.at("capture").get<std::string>());
      Json meta;std::ifstream(directory/"retrieval.json")>>meta;const auto o=meta.at("sensor_offset");Eigen::Vector3d offset(o[0].get<double>(),o[1].get<double>(),o[2].get<double>());
      Cloud query;if(pcl::io::loadPCDFile((directory/"query.pcd").string(),query)<0)throw std::runtime_error("query unavailable");
      std::vector<Cloud> independent;for(int i=0;i<count;++i){const auto path=directory/("independent_"+std::to_string(i)+".pcd");if(!std::filesystem::exists(path))break;Cloud scan;if(pcl::io::loadPCDFile(path.string(),scan)<0)throw std::runtime_error("independent scan unavailable");independent.push_back(std::move(scan));}
      const auto prepared=core.prepare(query,offset,[]{return false;});const auto initial=prepared.diagnostics();auto result=initial;const bool ready=prepared.ready();const bool confirm=ready&&independent.size()==size_t(count);
      if(confirm)result=core.confirm(prepared,independent,[]{return false;});
      Json row={{"id",job.at("id")},{"prepared_ready",ready},{"confirmation_replayed",confirm},{"confirmed",result.success},{"reason",result.reason},{"independent_count",independent.size()},{"prepared_candidate",candidateJson(initial.candidate)},{"last_candidate",candidateJson(result.candidate)},{"confirmations",Json::array()},{"prepare_sec",initial.retrieval_sec+initial.gicp_sec},{"confirmation_sec",result.confirmation_sec},{"wall_sec",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()}};
      for(const auto & c:result.confirmations)row["confirmations"].push_back(candidateJson(c));
      output["results"].push_back(row);std::ofstream(argv[4])<<output.dump(2)<<'\n';
      std::cout<<job.at("id")<<" ready="<<ready<<" confirmed="<<result.success<<" wall="<<row["wall_sec"]<<'\n'<<std::flush;
    }
    return 0;
  }catch(const std::exception & error){std::cerr<<error.what()<<'\n';return 2;}
}
