// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/visualization.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <thread>
#include <unistd.h>
using namespace global_relocalization;
TEST(Visualization,QualityFramesAndGenerationIsolation) {
  // This diagnostic-only test does not connect to the simulation or action server.
  setenv("ROS_DOMAIN_ID","189",1);rclcpp::init(0,nullptr);
  const auto file=std::filesystem::temp_directory_path()/("observer_test_"+std::to_string(getpid())+".pcd");
  Cloud map;for(int i=0;i<400;++i)map.push_back(pcl::PointXYZ(i*.1F,1.F,2.F));pcl::io::savePCDFileASCII(file.string(),map);
  {
    auto node=std::make_shared<rclcpp::Node>("observer_unit_test");Params params;
    Visualization visual(*node,file.string(),"map",params);
    std::vector<nlohmann::json> events;
    std::vector<visualization_msgs::msg::MarkerArray> markers;
    std::vector<sensor_msgs::msg::PointCloud2> raw,aligned;
    const auto qos=rclcpp::QoS(1).best_effort().transient_local();
    auto es=node->create_subscription<std_msgs::msg::String>("/global_relocalization/visual/events",rclcpp::QoS(32).best_effort(),[&](std_msgs::msg::String::ConstSharedPtr msg){events.push_back(nlohmann::json::parse(msg->data));});
    auto ms=node->create_subscription<visualization_msgs::msg::MarkerArray>("/global_relocalization/visual/markers",qos,[&](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg){markers.push_back(*msg);});
    auto rs=node->create_subscription<sensor_msgs::msg::PointCloud2>("/global_relocalization/visual/independent",qos,[&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){raw.push_back(*msg);});
    auto as=node->create_subscription<sensor_msgs::msg::PointCloud2>("/global_relocalization/visual/aligned_independent",qos,[&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){aligned.push_back(*msg);});
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    auto spin=[&](const std::function<bool()> & done) {const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);while(!done()&&std::chrono::steady_clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}return done();};
    auto first=visual.reset("processA_1");visual.drain();ASSERT_TRUE(spin([&]{return !markers.empty();}));
    Candidate c;c.converged=true;c.fitness=.01;c.inlier_ratio=1.;c.inliers=0;
    SearchEvent fine{"GICP_FINE_QUALITY_COMPLETE",0,1,c};visual.observe(fine,first);visual.drain();ASSERT_TRUE(spin([&]{return events.size()==1;}));
    EXPECT_FALSE(events.back()["quality_gates_passed"].get<bool>());EXPECT_FALSE(events.back()["quality_gates"]["inliers"].get<bool>());
    EXPECT_TRUE(events.back()["confirmation_consistency_passed"].is_null());
    ASSERT_TRUE(spin([&]{return std::any_of(markers.back().markers.begin(),markers.back().markers.end(),[](const auto & m){return m.id==10;});}));
    auto arrow=std::find_if(markers.back().markers.begin(),markers.back().markers.end(),[](const auto & m){return m.id==10;});EXPECT_FLOAT_EQ(arrow->color.r,1.);EXPECT_FLOAT_EQ(arrow->color.g,0.);
    fine.candidate.inliers=100;fine.candidate.fitness=std::numeric_limits<double>::quiet_NaN();visual.observe(fine,first);visual.drain();ASSERT_TRUE(spin([&]{return events.size()==2;}));
    EXPECT_FALSE(events.back()["quality_gates"]["finite_fitness"].get<bool>());EXPECT_FALSE(events.back()["quality_gates_passed"].get<bool>());
    SearchEvent coarse{"GICP_COARSE_SEED_COMPLETE",0,1,c};coarse.candidate.coarse_iterations=3;visual.observe(coarse,first);visual.drain();ASSERT_TRUE(spin([&]{return events.size()==3;}));
    EXPECT_EQ(events.back()["coarse_seed_state"],"usable");EXPECT_TRUE(events.back()["quality_gates_passed"].is_null());
    fine.candidate.fitness=.01;visual.observe(fine,first);visual.drain();ASSERT_TRUE(spin([&]{return events.size()==4;}));EXPECT_TRUE(events.back()["quality_gates_passed"].get<bool>());
    Cloud scan;scan.push_back(pcl::PointXYZ(1.,0.,0.));scan.push_back(pcl::PointXYZ(2.,0.,0.));
    SearchEvent confirmation{"CONFIRMATION_COMPLETE",0,3,fine.candidate,{},&scan};confirmation.candidate.map_from_reference.linear()=Eigen::AngleAxisd(1.57079632679,Eigen::Vector3d::UnitZ()).toRotationMatrix();confirmation.candidate.map_from_reference.translation()<<2.,3.,4.;confirmation.best_translation_delta=1.;
    visual.observe(confirmation,first);visual.drain();ASSERT_TRUE(spin([&]{return events.size()==5&&!aligned.empty()&&aligned.back().width==2;}));
    EXPECT_TRUE(events.back()["quality_gates_passed"].get<bool>());EXPECT_FALSE(events.back()["confirmation_consistency_passed"].get<bool>());
    EXPECT_EQ(raw.back().header.frame_id,"relocalization_reference");EXPECT_EQ(aligned.back().header.frame_id,"map");
    Cloud transformed;pcl::fromROSMsg(aligned.back(),transformed);EXPECT_NEAR(transformed[0].x,2.,1e-5);EXPECT_NEAR(transformed[0].y,4.,1e-5);EXPECT_NEAR(transformed[0].z,4.,1e-5);
    geometry_msgs::msg::PoseStamped accepted;accepted.pose.orientation.w=1.;visual.finish("SUCCEEDED",&accepted);const auto old_markers=markers.size();visual.drain();ASSERT_TRUE(spin([&]{return markers.size()>old_markers;}));
    EXPECT_TRUE(std::any_of(markers.back().markers.begin(),markers.back().markers.end(),[](const auto & m){return m.id==900;}));
    auto second=visual.reset("processA_2");visual.observe(fine,first);visual.query(scan,first,true);visual.stage("SETTLING");const auto before=markers.size();visual.drain();ASSERT_TRUE(spin([&]{return markers.size()>before;}));
    EXPECT_FALSE(std::any_of(markers.back().markers.begin(),markers.back().markers.end(),[](const auto & m){return m.id==900;}));
    EXPECT_TRUE(std::any_of(markers.back().markers.begin(),markers.back().markers.end(),[](const auto & m){return m.action==m.DELETEALL;}));
    EXPECT_EQ(events.size(),5u);visual.observe(fine,second);visual.drain();ASSERT_TRUE(spin([&]{return events.size()==6;}));EXPECT_EQ(events.back()["search_id"],"processA_2");EXPECT_EQ(events.back()["sequence"],1);
    visual.finish("FAILED");visual.observe(fine,second);visual.drain();executor.spin_some();EXPECT_EQ(events.size(),6u);
    executor.remove_node(node);
  }
  std::filesystem::remove(file);rclcpp::shutdown();
}
