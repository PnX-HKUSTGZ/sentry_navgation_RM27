// Isolated API contract: ordinary mode still waits for a true stop.
#include "experimental_frontend.hpp"
#include <random>
#include <iostream>
using namespace global_relocalization;
Cloud makeCloud(){Cloud c;std::mt19937 g(17);std::uniform_real_distribution<float>d(-3,3);for(int k=0;k<1200;++k)c.emplace_back(d(g),d(g),d(g));return c;}
bool unchanged(const nlohmann::json&a,const nlohmann::json&b){return a["last_accepted_stamp"]==b["last_accepted_stamp"]&&a["last_accepted_pose"]==b["last_accepted_pose"]&&a["submap_size"]==b["submap_size"];}
int main(){RawFrontendParams p;p.stationary_duration=2.;p.range_filter=p.deterministic=p.imu_rotation_prior=p.numerical_recovery=p.stationary_translation_prior=true;p.submap_scans=20;p.max_correspondence=.3;p.max_fitness=.18;p.max_iterations=1;p.retry_iterations=0;auto I=Eigen::Isometry3d::Identity();auto c=makeCloud();Cloud shifted=c;for(auto&q:shifted)q.x+=.025f;
for(int mode=0;mode<3;++mode){ExperimentalCalibratedFrontend f(p,I,I);bool refused=false;try{f.authorizeMappingNextRealScanRetry(1.);}catch(...){refused=true;}if(!refused)return 1;for(int k=0;k<=221;++k){RawImuSample s;s.stamp=k*.01;s.linear_acceleration<<0,0,9.81;f.addImu(s);}if(!f.processScan(c,2.).accepted)return 2;auto before=f.mappingRetryDiagnostics();auto failed=f.processScan(shifted,2.1);if(failed.accepted||failed.failure_class!="numerical_recoverable"||failed.rejected_gates!=std::vector<std::string>{"convergence"})return 3;auto after=f.mappingRetryDiagnostics();if(!unchanged(before,after)||after["mapping_tokens_consumed"]!=0)return 4;
if(mode==0){auto next=f.processScan(c,2.2);if(next.accepted||next.reason!="waiting for next scan acquired after recovery stop"||f.mappingRetryDiagnostics()["mapping_tokens_consumed"]!=0)return 5;continue;}
f.authorizeMappingNextRealScanRetry(2.1);auto authorized=f.mappingRetryDiagnostics();if(!unchanged(before,authorized)||authorized["real_stop_after_stamp"]!=-1||authorized["mapping_authorizations"]!=1)return 6;refused=false;try{f.authorizeMappingNextRealScanRetry(2.1);}catch(...){refused=true;}if(!refused)return 7;
auto next=f.processScan(mode==1?c:shifted,2.2);auto final=f.mappingRetryDiagnostics();if(final["mapping_tokens_consumed"]!=1||final["real_stop_after_stamp"]!=-1)return 8;if(mode==1){if(!next.accepted||final["recovered_scan_count"]!=1||final["last_accepted_stamp"]!=2.2)return 9;}else{if(next.accepted||!final["hard_fault"].get<bool>()||next.failure_class!="hard_fault")return 10;}
}
std::cout<<"mapping retry contracts passed: opt-in only, failed state uncommitted, one token, default requires stop, second failure hard-stops\n";return 0;}
