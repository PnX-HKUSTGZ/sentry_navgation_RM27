// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
// Scan Context descriptor implemented here from the published polar max-height
// definition (Kim and Kim, IROS 2018); no upstream Scan Context code is copied.
#include "global_relocalization/core.hpp"
#include "global_relocalization/bounded_kdtree.hpp"
#include "global_relocalization/raw_preprocessing.hpp"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <openssl/evp.h>
#include <omp.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <filesystem>
#include <atomic>
#include <future>
#include <mutex>
#include <nlohmann/json.hpp>

namespace global_relocalization {
namespace {
constexpr double pi = 3.14159265358979323846;
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now()-start).count(); }
Cloud::Ptr downsample(const Cloud & input, double leaf) {
  auto cleaned = std::make_shared<Cloud>();
  for (const auto & p: input) if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) cleaned->push_back(p);
  auto output = std::make_shared<Cloud>();
  pcl::VoxelGrid<pcl::PointXYZ> filter; filter.setInputCloud(cleaned);
  filter.setLeafSize(leaf, leaf, leaf); filter.filter(*output); return output;
}
Cloud translated(const Cloud & input, const Eigen::Vector3d & translation) {
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity(); t.translation()=translation;
  Cloud output; pcl::transformPointCloud(input,output,t.matrix().cast<float>()); return output;
}
double rotationDistance(const Eigen::Isometry3d & a,const Eigen::Isometry3d & b) {
  return Eigen::AngleAxisd(a.rotation().transpose()*b.rotation()).angle();
}
struct SearchCanceled {};
// Preserve upstream numerical reductions exactly. Cancellation is checked on
// the calling worker between optimizer stages, never thrown inside OpenMP.
struct CancelableReduction : small_gicp::ParallelReductionOMP {
  std::function<bool()> canceled;
  template<class... Args> auto linearize(Args&&... args) const {
    if(canceled())throw SearchCanceled{};
    return small_gicp::ParallelReductionOMP::linearize(std::forward<Args>(args)...);
  }
  template<class... Args> double error(Args&&... args) const {
    if(canceled())throw SearchCanceled{};
    return small_gicp::ParallelReductionOMP::error(std::forward<Args>(args)...);
  }
};
}

bool poseInSearchSide(const Eigen::Isometry3d & base_pose,const Params & params) {
  if(!base_pose.matrix().allFinite()||!std::isfinite(params.side_boundary_x))return false;
  const double x=base_pose.translation().x();
  if(params.search_side=="all")return true;
  if(params.search_side=="negative_x")return x<params.side_boundary_x;
  if(params.search_side=="positive_x")return x>params.side_boundary_x;
  throw std::invalid_argument("search_side must be all, negative_x or positive_x");
}

Descriptor scanContext(const Cloud & cloud, int rings, int sectors, double radius) {
  if (rings<1 || sectors<4 || radius<=0) throw std::invalid_argument("Invalid Scan Context dimensions");
  Descriptor d; d.cells=Eigen::MatrixXd::Zero(rings,sectors);
  for (const auto & p: cloud) {
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) continue;
    const double r=std::hypot(double(p.x),double(p.y)); if(r<0.2||r>=radius)continue;
    double angle=std::atan2(double(p.y),double(p.x));if(angle<0)angle+=2*pi;
    const int ring=std::min(rings-1,int(r/radius*rings));
    const int sector=std::min(sectors-1,int(angle/(2*pi)*sectors));
    d.cells(ring,sector)=std::max(d.cells(ring,sector),std::max(0.01,double(p.z)+3.0));
  }
  d.ring_key=d.cells.rowwise().mean(); return d;
}

std::pair<double,double> descriptorMatch(const Descriptor & q,const Descriptor & target) {
  if(q.cells.rows()!=target.cells.rows()||q.cells.cols()!=target.cells.cols())
    throw std::invalid_argument("Scan Context shape mismatch");
  const int sectors=q.cells.cols(); double best=1.0;int best_shift=0;
  for(int shift=0;shift<sectors;++shift) {
    double distance=0;int used=0;
    for(int s=0;s<sectors;++s) {
      const auto a=q.cells.col(s);const auto b=target.cells.col((s+shift)%sectors);
      const double an=a.norm(),bn=b.norm(); if(an==0&&bn==0)continue;
      ++used;distance+=(an==0||bn==0)?1.0:1.0-std::clamp(a.dot(b)/(an*bn),-1.0,1.0);
    }
    distance=used?distance/used:1.0;
    if(distance<best){best=distance;best_shift=shift;}
  }
  double yaw=best_shift*(2*pi/sectors);if(yaw>pi)yaw-=2*pi;
  return {best,yaw};
}

Eigen::Isometry3d composeCurrentBasePose(const Eigen::Isometry3d & map_from_reference,
  const Eigen::Isometry3d & odom_from_reference,const Eigen::Isometry3d & odom_from_current) {
  return map_from_reference*odom_from_reference.inverse()*odom_from_current;
}
Eigen::Isometry3d basePoseFromSensorView(const Eigen::Vector3d & sensor_in_map,
  const Eigen::Vector3d & sensor_in_base,double base_yaw) {
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
  pose.linear()=Eigen::AngleAxisd(base_yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation()=sensor_in_map-pose.rotation()*sensor_in_base;
  return pose;
}

std::string sha256File(const std::string & path) {
  std::ifstream file(path,std::ios::binary);if(!file)throw std::runtime_error("Cannot read map for SHA256");
  auto context=EVP_MD_CTX_new();if(!context)throw std::runtime_error("Cannot allocate SHA256 context");
  EVP_DigestInit_ex(context,EVP_sha256(),nullptr);char buffer[65536];
  while(file){file.read(buffer,sizeof(buffer));EVP_DigestUpdate(context,buffer,file.gcount());}
  unsigned char digest[EVP_MAX_MD_SIZE];unsigned int length=0;
  EVP_DigestFinal_ex(context,digest,&length);EVP_MD_CTX_free(context);
  std::ostringstream result;for(unsigned int i=0;i<length;++i)result<<std::hex<<std::setw(2)<<std::setfill('0')<<int(digest[i]);
  return result.str();
}

std::vector<Candidate> selectRetrievalCandidates(std::vector<Candidate> candidates,int context_count,int ring_count) {
  if(context_count<1||ring_count<0)throw std::invalid_argument("invalid retrieval candidate counts");
  std::sort(candidates.begin(),candidates.end(),[](const Candidate&a,const Candidate&b){return a.descriptor_distance<b.descriptor_distance;});
  for(size_t i=0;i<candidates.size();++i)candidates[i].scan_context_rank=i+1;
  std::vector<Candidate> selected;
  for(size_t i=0;i<std::min(candidates.size(),size_t(context_count));++i){auto c=candidates[i];c.from_scan_context=true;selected.push_back(c);}
  std::sort(candidates.begin(),candidates.end(),[](const Candidate&a,const Candidate&b){return a.ring_key_rank<b.ring_key_rank;});
  for(size_t i=0;i<std::min(candidates.size(),size_t(ring_count));++i) {
    const auto&c=candidates[i];auto duplicate=std::find_if(selected.begin(),selected.end(),[&](const Candidate&other){return other.ring_key_rank==c.ring_key_rank;});
    if(duplicate!=selected.end())duplicate->from_ring_key=true;
    else{auto added=c;added.from_ring_key=true;selected.push_back(added);}
  }
  return selected;
}

namespace {
struct SearchProvenance { size_t revision{0}; };
}
struct PreparedSearch::State {
  explicit State(SearchResult result):diagnostics(std::move(result)) {}
  std::weak_ptr<SearchProvenance> owner;
  size_t revision{0};
  bool prepared{false}, consumed{false};
  const SearchResult diagnostics;
  std::string diagnostic_directory;
};
bool PreparedSearch::ready() const {
  if(!state_||!state_->prepared||state_->consumed)return false;
  const auto owner=state_->owner.lock();
  return owner&&owner->revision==state_->revision;
}
const SearchResult & PreparedSearch::diagnostics() const {
  static const SearchResult empty;
  return state_?state_->diagnostics:empty;
}

struct Core::Impl {
  explicit Impl(const Params & p):params(p) {}
  Params params;Cloud::Ptr map;
  std::shared_ptr<SearchProvenance> provenance=std::make_shared<SearchProvenance>();
  std::string hash;size_t diagnostic_search_count{0};
  struct View { Eigen::Vector3d base;Descriptor descriptor; };
  std::vector<View> database;
  Eigen::Vector3d database_offset=Eigen::Vector3d::Constant(1e9);
  pcl::PointCloud<pcl::PointCovariance>::Ptr target,coarse_target;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> coarse_tree;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> tree;
  // FLANN's shared-query concurrency is not assumed. Build once per worker
  // at map load; each query owns its indices/distances and Registration.
  std::vector<std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>>> quality_trees;

  Cloud visible(const Eigen::Vector3d & origin) const {
    // Fixed spherical z-buffer: all map points are tested, then returns behind
    // the nearest measured surface in the same angular bin are removed.
    constexpr int azimuth_bins=720,elevation_bins=180;
    std::vector<float> nearest(azimuth_bins*elevation_bins,std::numeric_limits<float>::infinity());
    struct Sample { pcl::PointXYZ p;int bin;float range; };
    std::vector<Sample> samples;samples.reserve(map->size());
    for(const auto & p:*map) {
      const Eigen::Vector3d v=Eigen::Vector3d(p.x,p.y,p.z)-origin;
      const double range=v.norm();if(range<0.5||range>params.db_max_radius)continue;
      double az=std::atan2(v.y(),v.x());if(az<0)az+=2*pi;
      const double el=std::asin(std::clamp(v.z()/range,-1.,1.));
      if(el<params.db_min_elevation_deg*pi/180.||el>params.db_max_elevation_deg*pi/180.)continue;
      const int a=std::min(azimuth_bins-1,int(az/(2*pi)*azimuth_bins));
      const int e=std::clamp(int((el+pi/2)/pi*elevation_bins),0,elevation_bins-1);
      const int bin=e*azimuth_bins+a; nearest[bin]=std::min(nearest[bin],float(range));
      samples.push_back({p,bin,float(range)});
    }
    Cloud out;for(const auto & s:samples)if(s.range<=nearest[s.bin]+0.25F)
      out.push_back(pcl::PointXYZ(s.p.x-origin.x(),s.p.y-origin.y(),s.p.z-origin.z()));
    return out;
  }

  pcl::PointCloud<pcl::PointCovariance>::Ptr covarianceCloud(const Cloud & input,double leaf,int threads=4) const {
    if(!params.deterministic_preprocessing)
      return small_gicp::voxelgrid_sampling_omp<Cloud,pcl::PointCloud<pcl::PointCovariance>>(input,leaf,threads);
    const auto points=deterministicVoxel(input,leaf);
    auto cloud=std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();cloud->resize(points->size());
    for(size_t i=0;i<points->size();++i) {
      auto & point=(*cloud)[i];const auto & xyz=(*points)[i];
      point.getVector4fMap()<<xyz.x,xyz.y,xyz.z,1.F;point.cov.setZero();
    }
    return cloud;
  }
  struct PreparedSource {
    pcl::PointCloud<pcl::PointCovariance>::Ptr fine,coarse;
  };
  // Source coordinates and covariances are independent of the candidate seed.
  // Keep these local to one search; each stopped scan is prepared separately.
  PreparedSource prepareSource(const Cloud & input,bool coarse) const {
    PreparedSource prepared;
    prepared.fine=covarianceCloud(input,params.gicp_leaf_size);
    if(prepared.fine&&prepared.fine->size()>=20)
      small_gicp::estimate_covariances_omp(*prepared.fine,20,4);
    if(coarse) {
      prepared.coarse=covarianceCloud(input,std::max(.20,params.gicp_leaf_size));
      if(prepared.coarse&&prepared.coarse->size()>=20)
        small_gicp::estimate_covariances_omp(*prepared.coarse,20,4);
    }
    return prepared;
  }
  Candidate refine(const PreparedSource & prepared,const Eigen::Isometry3d & guess,
    const std::function<bool()> & canceled,size_t worker=0,bool coarse=false,
    const std::function<void(const Candidate &)> & coarse_observer={}) {
    const auto began=Clock::now();
    Candidate candidate;candidate.map_from_reference=guess;
    const auto & source=prepared.fine;
    candidate.source_points=source?source->size():0;
    candidate.coarse_source_points=coarse&&prepared.coarse?prepared.coarse->size():0;
    if(!source||source->size()<20)return candidate;
    try {
    if(canceled())throw SearchCanceled{};
    small_gicp::Registration<small_gicp::GICPFactor,CancelableReduction> registration;
    registration.reduction.canceled=canceled;
    registration.reduction.num_threads=4;
    registration.rejector.max_dist_sq=params.gicp_max_correspondence*params.gicp_max_correspondence;
    registration.optimizer.max_iterations=params.gicp_max_iterations;
    Eigen::Isometry3d seed=guess;
    if(coarse) {
      Candidate coarse_result=candidate;
      const auto & coarse_source=prepared.coarse;
      if(coarse_source&&coarse_source->size()>=20) {
        registration.rejector.max_dist_sq=std::pow(std::max(1.5,params.gicp_max_correspondence),2);
        registration.optimizer.max_iterations=std::min(30,params.gicp_max_iterations);
        const auto coarse_start=Clock::now();
        const BoundedKdTree bounded_coarse(*coarse_tree,registration.rejector.max_dist_sq);
        const auto initial=params.bounded_global_correspondences?
          registration.align(*coarse_target,*coarse_source,bounded_coarse,guess):
          registration.align(*coarse_target,*coarse_source,*coarse_tree,guess);
        candidate.coarse_sec=seconds(coarse_start);candidate.coarse_iterations=initial.iterations+1;
        coarse_result.map_from_reference=initial.T_target_source;coarse_result.converged=initial.converged;
        coarse_result.coarse_sec=candidate.coarse_sec;coarse_result.coarse_iterations=candidate.coarse_iterations;
        if(initial.T_target_source.matrix().allFinite())seed=initial.T_target_source;
        registration.rejector.max_dist_sq=params.gicp_max_correspondence*params.gicp_max_correspondence;
        registration.optimizer.max_iterations=params.gicp_max_iterations;
      }
      if(coarse_observer)coarse_observer(coarse_result);
    }
    if(canceled())throw SearchCanceled{};
    const auto fine_start=Clock::now();
    const BoundedKdTree bounded_fine(*tree,registration.rejector.max_dist_sq);
    const auto result=params.bounded_global_correspondences?
      registration.align(*target,*source,bounded_fine,seed):
      registration.align(*target,*source,*tree,seed);
    candidate.fine_sec=seconds(fine_start);candidate.fine_iterations=result.iterations+1;
    candidate.converged=result.converged;
    candidate.map_from_reference=result.T_target_source;
    if(!candidate.map_from_reference.matrix().allFinite())return candidate;
    candidate.within_search_side=poseInSearchSide(candidate.map_from_reference,params);
    if(canceled())throw SearchCanceled{};
    const auto quality_start=Clock::now();
    auto & quality_tree=*quality_trees.at(worker);
    std::vector<int> indices(1);std::vector<float> distances(1);double sum=0;size_t count=0,visited=0;
    const double maxsq=params.gicp_max_correspondence*params.gicp_max_correspondence;
    for(const auto & p:*source) {
      if((visited++%256)==0&&canceled())throw SearchCanceled{};
      const Eigen::Vector3d v=result.T_target_source*Eigen::Vector3d(p.x,p.y,p.z);
      pcl::PointXYZ transformed(v.x(),v.y(),v.z());
      if(quality_tree.nearestKSearch(transformed,1,indices,distances)>0&&distances[0]<=maxsq)
      {sum+=distances[0];++count;}
    }
    candidate.inliers=count;candidate.inlier_ratio=double(count)/source->size();
    candidate.fitness=count?std::sqrt(sum/count):1e9;
    candidate.quality_sec=seconds(quality_start);
    }catch(const SearchCanceled &){candidate.canceled=true;}
    candidate.total_sec=seconds(began);
    return candidate;
  }
  bool accepted(const Candidate & c) const {
    return c.within_search_side&&c.converged&&c.inliers>=20&&std::isfinite(c.fitness)&&
      c.fitness<=params.max_fitness&&c.inlier_ratio>=params.min_inlier_ratio;
  }
};

Core::Core(const Params & params):impl_(std::make_unique<Impl>(params)) {
  if(params.db_grid_resolution<=0||params.gicp_leaf_size<=0||params.top_k<1||params.ring_key_candidates<0||params.ring_key_candidates>64||params.candidate_workers<1||params.candidate_workers>2||params.minimum_confirmation_scans<1)
    throw std::invalid_argument("Invalid database/registration parameters");
  if((params.search_side!="all"&&params.search_side!="negative_x"&&params.search_side!="positive_x")||
    !std::isfinite(params.side_boundary_x))throw std::invalid_argument("Invalid half-field search prior");
  omp_set_num_threads(4);
}
Core::~Core()=default;
void Core::loadMap(const std::string & path) {
  ++impl_->provenance->revision;
  Cloud input;if(pcl::io::loadPCDFile(path,input)<0||input.empty())throw std::runtime_error("Cannot load nonempty prior PCD: "+path);
  impl_->hash=sha256File(path);
  impl_->map=impl_->params.deterministic_preprocessing?deterministicVoxel(input,impl_->params.gicp_leaf_size):downsample(input,impl_->params.gicp_leaf_size);
  if(impl_->map->size()<20)throw std::runtime_error("Prior PCD has too few finite points");
  impl_->target=impl_->covarianceCloud(*impl_->map,impl_->params.gicp_leaf_size);
  small_gicp::estimate_covariances_omp(*impl_->target,20,4);
  impl_->tree=std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(impl_->target,small_gicp::KdTreeBuilderOMP(4));
  impl_->coarse_target=impl_->covarianceCloud(*impl_->map,std::max(.20,impl_->params.gicp_leaf_size));
  small_gicp::estimate_covariances_omp(*impl_->coarse_target,20,4);
  impl_->coarse_tree=std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(impl_->coarse_target,small_gicp::KdTreeBuilderOMP(4));
  impl_->quality_trees.clear();
  for(int i=0;i<impl_->params.candidate_workers;++i){auto quality=std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();quality->setInputCloud(impl_->map);impl_->quality_trees.push_back(std::move(quality));}
  impl_->database.clear();
}
void Core::buildDatabase(const Eigen::Vector3d & offset) {
  if(!impl_->map)throw std::runtime_error("Map must be loaded before database construction");
  if(!impl_->database.empty()&&(impl_->database_offset-offset).norm()<1e-6)return;
  ++impl_->provenance->revision;
  impl_->database.clear();impl_->database_offset=offset;
  pcl::PointXYZ lo,hi;pcl::getMinMax3D(*impl_->map,lo,hi);const auto & p=impl_->params;
  for(double x=lo.x;x<=hi.x;x+=p.db_grid_resolution)for(double y=lo.y;y<=hi.y;y+=p.db_grid_resolution) {
    const Eigen::Vector3d base(x,y,p.db_base_z);
    // A view is sensor-centered. Keep a mount-sized boundary halo in the
    // database, then check the actual yaw-dependent base seed below. This
    // removes the opposite half without confusing sensor x with base x.
    const double sensor_x=base.x()+offset.x(),mount_radius=std::hypot(offset.x(),offset.y());
    if(p.search_side=="negative_x"&&sensor_x-mount_radius>=p.side_boundary_x)continue;
    if(p.search_side=="positive_x"&&sensor_x+mount_radius<=p.side_boundary_x)continue;
    Cloud visible=impl_->visible(base+offset);
    if(int(visible.size())<p.db_min_points)continue;
    impl_->database.push_back({base,scanContext(visible,p.sc_rings,p.sc_sectors,p.db_max_radius)});
  }
  if(impl_->database.empty())throw std::runtime_error("No usable virtual views in prior PCD");
}
const std::string & Core::mapHash()const{return impl_->hash;}
size_t Core::databaseSize()const{return impl_->database.size();}
SearchResult Core::search(const Cloud & query,const Cloud & independent,const Eigen::Vector3d & offset,
  const std::function<bool()> & canceled, const SearchObserver & observer) {
  return search(query,std::vector<Cloud>{independent},offset,canceled,observer);
}
SearchResult Core::search(const Cloud & query,const std::vector<Cloud> & independent,const Eigen::Vector3d & offset,
  const std::function<bool()> & canceled, const SearchObserver & observer) {
  // Preserve the legacy early count gate, even when no prior map is loaded.
  if(independent.size()<impl_->params.minimum_confirmation_scans) {
    ++impl_->provenance->revision;
    SearchResult out;out.map_hash=impl_->hash;out.candidate_workers=impl_->params.candidate_workers;
    out.reason=independent.empty()?"no independent stopped scan provided":"insufficient independent stopped scans";
    return out;
  }
  const auto prepared=prepare(query,offset,canceled,observer);
  if(!prepared.ready()) {
    // Legacy offline diagnostics include the supplied stopped scans even when
    // global retrieval/ambiguity fails before confirmation can begin.
    if(!prepared.state_->diagnostic_directory.empty()) {
      const auto directory=std::filesystem::path(prepared.state_->diagnostic_directory);
      for(size_t i=0;i<independent.size();++i)
        pcl::io::savePCDFileBinary((directory/("independent_"+std::to_string(i)+".pcd")).string(),independent[i]);
    }
    return prepared.diagnostics();
  }
  return confirm(prepared,independent,canceled,observer);
}
PreparedSearch Core::prepare(const Cloud & query,const Eigen::Vector3d & offset,
  const std::function<bool()> & canceled, const SearchObserver & observer) {
  ++impl_->provenance->revision;
  bool ready=false;
  std::string diagnostic_directory;
  const auto run=[&]() -> SearchResult {
  std::mutex observer_mutex;
  const auto notify=[&](const SearchEvent & event) {
    if(!observer)return;
    std::lock_guard<std::mutex> lock(observer_mutex);
    try {observer(event);} catch(...) {} // Diagnostic failures never change search results.
  };
  SearchResult out;out.map_hash=impl_->hash;out.candidate_workers=impl_->params.candidate_workers;auto start=Clock::now();
  // Serialize callbacks so even a stateful caller predicate is never raced.
  std::mutex cancel_mutex;
  const auto checkedCancel=[&]{std::lock_guard<std::mutex> lock(cancel_mutex);return canceled();};
  omp_set_num_threads(4);
  buildDatabase(offset);out.database_size=impl_->database.size();
  if(checkedCancel()){out.reason="canceled before retrieval";return out;}
  const auto & p=impl_->params;
  const Descriptor descriptor=scanContext(translated(query,-offset),p.sc_rings,p.sc_sectors,p.db_max_radius);
  std::vector<std::pair<double,size_t>> shortlist;
  for(size_t i=0;i<impl_->database.size();++i)
    shortlist.emplace_back((descriptor.ring_key-impl_->database[i].descriptor.ring_key).squaredNorm(),i);
  std::sort(shortlist.begin(),shortlist.end());
  const size_t shortlist_limit=p.retrieval_all_views?shortlist.size():std::min(shortlist.size(),size_t(std::max({40,p.top_k*10,p.ring_key_candidates})));
  // Optional offline diagnostics never consume a pose or influence retrieval.
  if(!p.diagnostic_output_dir.empty()) {
    const auto directory=std::filesystem::path(p.diagnostic_output_dir)/("search_"+std::to_string(++impl_->diagnostic_search_count));
    std::filesystem::create_directories(directory);
    diagnostic_directory=directory.string();
    pcl::io::savePCDFileBinary((directory/"query.pcd").string(),query);
    nlohmann::json data={{"map_sha256",impl_->hash},{"query_points",query.size()},
      {"sensor_offset",{offset.x(),offset.y(),offset.z()}},{"shortlist_limit",shortlist_limit},{"top_k",p.top_k},{"ring_key_candidates",p.ring_key_candidates},{"retrieval_all_views",p.retrieval_all_views},{"views",nlohmann::json::array()}};
    for(size_t rank=0;rank<shortlist.size();++rank) {
      const auto & item=shortlist[rank];const auto & view=impl_->database[item.second];
      const auto match=descriptorMatch(descriptor,view.descriptor);
      const auto seed=basePoseFromSensorView(view.base+offset,offset,match.second);
      data["views"].push_back({{"database_index",item.second},{"ring_rank",rank+1},{"in_shortlist",rank<shortlist_limit},{"ring_distance",item.first},
        {"descriptor_distance",match.first},{"yaw",match.second},{"view_base",{view.base.x(),view.base.y(),view.base.z()}},
        {"seed_xyz",{seed.translation().x(),seed.translation().y(),seed.translation().z()}},{"seed_in_search_side",poseInSearchSide(seed,p)}});
    }
    std::ofstream output(directory/"retrieval.json");output<<data.dump(2);
  }
  shortlist.resize(shortlist_limit);
  std::vector<Candidate> guesses;
  size_t ring_rank=0;
  for(const auto & item:shortlist) {
    ++ring_rank;const auto match=descriptorMatch(descriptor,impl_->database[item.second].descriptor);
    Candidate c;c.descriptor_distance=match.first;c.ring_key_rank=ring_rank;
    c.map_from_reference=basePoseFromSensorView(impl_->database[item.second].base+offset,offset,match.second);
    if(!poseInSearchSide(c.map_from_reference,p))continue;
    c.seed_translation=c.map_from_reference.translation();c.seed_yaw=match.second;guesses.push_back(c);
  }
  out.retrieval_scored_views=shortlist.size();
  std::sort(guesses.begin(),guesses.end(),[](const Candidate & a,const Candidate & b){return a.descriptor_distance<b.descriptor_distance;});
  for(size_t i=0;i<guesses.size();++i)guesses[i].scan_context_rank=i+1;
  const auto retrieval_pool=guesses;
  guesses=selectRetrievalCandidates(guesses,p.top_k,p.ring_key_candidates);
  out.retrieval_candidate_count=guesses.size();
  if(observer)notify({"RETRIEVAL_COMPLETE",0,guesses.size(),{},guesses,nullptr});
  out.retrieval_ring_added=std::count_if(guesses.begin(),guesses.end(),[](const Candidate & c){return c.from_ring_key&&!c.from_scan_context;});
  out.retrieval_sec=seconds(start);start=Clock::now();
  if(guesses.empty()){out.reason="no retrieved base-pose seed lies inside configured search side";return out;}
  const auto query_source=impl_->prepareSource(query,true);++out.query_preparations;
  if(checkedCancel()){out.gicp_sec=seconds(start);out.reason="canceled preparing GICP";return out;}
  std::vector<Candidate> refined(guesses.size());
  std::vector<unsigned char> started(guesses.size(),0);
  std::atomic<size_t> next{0};std::atomic_bool stop{false};
  const auto evaluate=[&](size_t worker) {
    omp_set_num_threads(4);
    while(!stop.load()) {
      if(checkedCancel()){stop.store(true);break;}
      const size_t index=next.fetch_add(1);if(index>=guesses.size())break;
      const auto & guess=guesses[index];
      if(observer)notify({"GICP_CANDIDATE_START",index,guesses.size(),guess,{},nullptr});
      std::function<void(const Candidate &)> coarse_observer;
      if(observer)coarse_observer=[&](const Candidate & result) {
        auto observed=result;observed.scan_context_rank=guess.scan_context_rank;observed.ring_key_rank=guess.ring_key_rank;
        notify({"GICP_COARSE_SEED_COMPLETE",index,guesses.size(),observed,{},nullptr});
      };
      auto c=impl_->refine(query_source,guess.map_from_reference,checkedCancel,worker,true,coarse_observer);
      c.descriptor_distance=guess.descriptor_distance;
      c.ring_key_rank=guess.ring_key_rank;c.scan_context_rank=guess.scan_context_rank;
      c.from_scan_context=guess.from_scan_context;c.from_ring_key=guess.from_ring_key;
      c.seed_translation=guess.seed_translation;c.seed_yaw=guess.seed_yaw;
      refined[index]=std::move(c);started[index]=1;
      if(observer)notify({"GICP_FINE_QUALITY_COMPLETE",index,guesses.size(),refined[index],{},nullptr});
      if(refined[index].canceled)stop.store(true);
    }
  };
  if(p.candidate_workers==1)evaluate(0);
  else {
    // Futures join on scope exit, including exceptions. No detached work can
    // outlive the prepared sources, map trees or caller cancellation predicate.
    std::vector<std::future<void>> tasks;
    for(int worker=0;worker<p.candidate_workers;++worker)
      tasks.push_back(std::async(std::launch::async,[&,worker]{evaluate(worker);}));
    for(auto & task:tasks)task.get();
  }
  // Aggregate only after joining, in original retrieval order. Every seed is
  // evaluated before quality ranking/ambiguity; completion order has no vote.
  std::vector<Candidate> accepted;
  for(size_t index=0;index<guesses.size();++index)if(started[index]) {
    const auto & c=refined[index];++out.candidates_tested;out.hypotheses.push_back(c);
    if(!c.canceled&&c.fitness<out.candidate.fitness)out.candidate=c;
    if(!c.canceled&&impl_->accepted(c))accepted.push_back(c);
  }
  out.gicp_sec=seconds(start);
  if(stop.load()||checkedCancel()){out.reason="canceled during GICP";return out;}
  // The first page can miss a repetitive place. Search the next equal page
  // only after every initial seed has failed the quality gates.
  if(accepted.empty()) {
    std::vector<size_t> tested;for(const auto & g:guesses)tested.push_back(g.scan_context_rank);
    std::vector<Candidate> extra;
    for(const auto & c:retrieval_pool) {
      if(c.scan_context_rank<=size_t(p.top_k)||c.scan_context_rank>size_t(p.top_k*2))continue;
      if(std::find(tested.begin(),tested.end(),c.scan_context_rank)!=tested.end())continue;
      auto added=c;added.from_scan_context=true;added.from_ring_key=false;extra.push_back(added);
    }
    if(!extra.empty()) {
      guesses=std::move(extra);out.retrieval_candidate_count+=guesses.size();
      refined.assign(guesses.size(),{});started.assign(guesses.size(),0);next.store(0);stop.store(false);
      if(p.candidate_workers==1)evaluate(0);
      else {
        std::vector<std::future<void>> tasks;
        for(int worker=0;worker<p.candidate_workers;++worker)
          tasks.push_back(std::async(std::launch::async,[&,worker]{evaluate(worker);}));
        for(auto & task:tasks)task.get();
      }
      for(size_t index=0;index<guesses.size();++index)if(started[index]) {
        const auto & c=refined[index];++out.candidates_tested;out.hypotheses.push_back(c);
        if(!c.canceled&&c.fitness<out.candidate.fitness)out.candidate=c;
        if(!c.canceled&&impl_->accepted(c))accepted.push_back(c);
      }
      out.gicp_sec=seconds(start);
    }
  }
  if(stop.load()||checkedCancel()){out.reason="canceled during GICP";return out;}
  if(accepted.empty()){out.reason="no retrieved candidate passed GICP quality and search-side gates";return out;}
  std::sort(accepted.begin(),accepted.end(),[](const Candidate & a,const Candidate & b){return a.fitness<b.fitness;});
  const auto & best=accepted.front();out.candidate=best;
  for(size_t i=1;i<accepted.size();++i) {
    const auto & alternative=accepted[i];
    const bool different=(alternative.map_from_reference.translation()-best.map_from_reference.translation()).norm()>0.75||rotationDistance(alternative.map_from_reference,best.map_from_reference)>0.34906585;
    if(different&&alternative.fitness<=best.fitness*(1.+p.ambiguity_margin)+0.005) {
      out.reason="ambiguous distinct global registration hypotheses";return out;
    }
  }
  if(checkedCancel()){out.reason="canceled before independent confirmation";return out;}
  ready=true;out.reason="global registration prepared; independent stopped scans required";return out;
  };
  PreparedSearch prepared;prepared.state_=std::make_shared<PreparedSearch::State>(run());
  prepared.state_->owner=impl_->provenance;prepared.state_->revision=impl_->provenance->revision;
  prepared.state_->prepared=ready;prepared.state_->diagnostic_directory=std::move(diagnostic_directory);
  return prepared;
}
SearchResult Core::confirm(const PreparedSearch & prepared,const std::vector<Cloud> & independent,
  const std::function<bool()> & canceled, const SearchObserver & observer) {
  SearchResult out=prepared.diagnostics();
  if(!prepared.ready()||prepared.state_->owner.lock()!=impl_->provenance) {
    out.success=false;out.reason="invalid, stale, consumed or foreign prepared search";return out;
  }
  // Consume before every gate, including cancellation and insufficient scans.
  // Copies of this token can never restart a canceled or failed confirmation.
  prepared.state_->consumed=true;
  if(independent.empty()){out.reason="no independent stopped scan provided";return out;}
  if(independent.size()<impl_->params.minimum_confirmation_scans){out.reason="insufficient independent stopped scans";return out;}
  const auto checkedCancel=[&]{return canceled();};
  const auto notify=[&](const SearchEvent & event) {
    if(observer)try {observer(event);} catch(...) {}
  };
  const auto & p=impl_->params;
  const Candidate best=out.candidate;
  if(!prepared.state_->diagnostic_directory.empty()) {
    const auto directory=std::filesystem::path(prepared.state_->diagnostic_directory);
    for(size_t i=0;i<independent.size();++i)
      pcl::io::savePCDFileBinary((directory/("independent_"+std::to_string(i)+".pcd")).string(),independent[i]);
  }
  auto start=Clock::now();
  if(checkedCancel()){out.reason="canceled before independent confirmation";return out;}
  start=Clock::now();Candidate confirmation=best;std::vector<Candidate> confirmed;
  for(size_t confirmation_index=0;confirmation_index<independent.size();++confirmation_index) {
    const auto & scan=independent[confirmation_index];
    if(observer)notify({"CONFIRMATION_START",confirmation_index,independent.size(),best,{},&scan});
    if(checkedCancel()){out.confirmation_sec=seconds(start);out.reason="canceled during independent confirmation";return out;}
    const auto stopped_source=impl_->prepareSource(scan,false);++out.independent_preparations;
    confirmation=impl_->refine(stopped_source,best.map_from_reference,checkedCancel);
    out.confirmations.push_back(confirmation);
    if(observer) {
      SearchEvent event{"CONFIRMATION_COMPLETE",confirmation_index,independent.size(),confirmation,{},&scan};
      event.best_translation_delta=(confirmation.map_from_reference.translation()-best.map_from_reference.translation()).norm();
      event.best_rotation_delta=rotationDistance(confirmation.map_from_reference,best.map_from_reference);
      for(const auto & previous:confirmed) {
        event.previous_translation_delta=std::max(event.previous_translation_delta,(confirmation.map_from_reference.translation()-previous.map_from_reference.translation()).norm());
        event.previous_rotation_delta=std::max(event.previous_rotation_delta,rotationDistance(confirmation.map_from_reference,previous.map_from_reference));
      }
      notify(event);
    }
    if(confirmation.canceled||checkedCancel()){out.confirmation_sec=seconds(start);out.reason="canceled during independent confirmation";return out;}
  if(!impl_->accepted(confirmation)){out.confirmation_sec=seconds(start);out.reason="independent stopped scan failed quality or search-side gates";return out;}
  if((confirmation.map_from_reference.translation()-best.map_from_reference.translation()).norm()>p.confirmation_max_translation||
    rotationDistance(confirmation.map_from_reference,best.map_from_reference)>p.confirmation_max_rotation) {
    out.confirmation_sec=seconds(start);out.reason="independent stopped scan disagrees with accumulated registration";return out;
  }
    for(const auto & previous:confirmed) {
      if((confirmation.map_from_reference.translation()-previous.map_from_reference.translation()).norm()>p.confirmation_max_translation||rotationDistance(confirmation.map_from_reference,previous.map_from_reference)>p.confirmation_max_rotation){out.confirmation_sec=seconds(start);out.reason="independent stopped scans disagree with each other";return out;}
    }
    confirmed.push_back(confirmation);
  }
  out.confirmation_sec=seconds(start);
  if(checkedCancel()){out.reason="canceled after independent confirmation";return out;}
  out.candidate=confirmation;out.success=true;out.reason="retrieval and independent stopped-scan registration accepted";return out;
}
}
