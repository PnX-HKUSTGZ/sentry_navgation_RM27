#include <gtest/gtest.h>
#include <rog_map/prior_dynamic_obstacles.hpp>
#include <rog_map/prior_map.hpp>
#include <cmath>

namespace {
using Detector = rog_map::PriorDynamicObstacles;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
Detector::Cloud background() {
  Detector::Cloud map;
  for (int x = -40; x <= 80; ++x) for (int y = -30; y <= 30; ++y)
    map.emplace_back(x * .05F, y * .05F, 0.0F);
  for (int y = -8; y <= 8; ++y) for (int z = 4; z <= 18; ++z)
    map.emplace_back(3.0F, y * .05F, z * .05F);
  return map;
}
Cloud vehicle(float x = 1.2F) {
  Cloud cloud;
  for (int y = -4; y <= 4; ++y) for (int z = 5; z <= 15; ++z) {
    pcl::PointXYZI p; p.x=x; p.y=y*.05F; p.z=z*.05F; cloud.push_back(p);
  }
  return cloud;
}
void update(Detector &d, const Cloud &cloud, double t) {
  d.update(cloud, Eigen::Isometry3d::Identity(), {0, 0, .5}, {0, 0, 0}, t);
}
TEST(PriorVehicleDetection, ParkedAtFirstObservationAndAfterStoppingStillBlocks) {
  Detector d; d.setBackground(background());
  for (double t : {1., 1.1, 4., 8., 12.}) {
    update(d, vehicle(), t);
    ASSERT_EQ(d.tracks().size(), 1U);
    EXPECT_TRUE(d.occupied({1.2, 0}, .025, t));
  }
}
TEST(PriorVehicleDetection, MissingFovRetainsTargetUntilHoldExpires) {
  Detector d; d.setBackground(background()); update(d, vehicle(), 1.0);
  update(d, {}, 5.2); // One full 4.19 s gimbal revolution.
  EXPECT_TRUE(d.occupied({1.2, 0}, .025, 5.2));
  update(d, {}, 7.1);
  EXPECT_FALSE(d.occupied({1.2, 0}, .025, 7.1));
}
TEST(PriorVehicleDetection, PositiveRaysClearDepartedVehicle) {
  Detector d; auto map=background(); d.setBackground(map); update(d, vehicle(), 1.0);
  Cloud rays;
  for (const auto &p : map) if (p.x == 3.0F && p.z > .25F) {
    pcl::PointXYZI q; q.x=p.x; q.y=p.y; q.z=p.z; rays.push_back(q);
  }
  update(d, rays, 1.1); EXPECT_EQ(d.tracks().size(), 1U);
  update(d, rays, 1.2); EXPECT_TRUE(d.tracks().empty());
}
TEST(PriorVehicleDetection, MovingSensorDoesNotTurnFixedSceneIntoVehicles) {
  Detector d; auto map=background(); d.setBackground(map);
  for (double angle : {0., .5, 1.5, 3.0}) {
    Eigen::Isometry3d transform=Eigen::Isometry3d::Identity();
    transform.linear()=Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform.translation()=Eigen::Vector3d(.2, .1, .0);
    Cloud cloud;
    for (const auto &p : map) {
      const Eigen::Vector3d v=transform.inverse()*Eigen::Vector3d(p.x,p.y,p.z);
      pcl::PointXYZI q; q.x=v.x();q.y=v.y();q.z=v.z();cloud.push_back(q);
    }
    d.update(cloud,transform,{.2,.1,.5},{.2,.1,0},1.0+angle);
    EXPECT_TRUE(d.tracks().empty());
  }
}
TEST(PriorVehicleDetection, MovingObservationsUpdatePositionAndStoppedTargetPersists) {
  Detector d; d.setBackground(background());
  update(d, vehicle(1.2), 1.0); update(d, vehicle(1.3), 1.2);
  ASSERT_EQ(d.tracks().size(),1U);
  EXPECT_TRUE(d.occupied({1.64,0},.025,1.2));
  for(int i=0;i<20;++i) update(d,vehicle(1.3),1.3+.1*i);
  EXPECT_TRUE(d.occupied({1.3,0},.025,3.2));
}
TEST(PriorVehicleDetection, NewOccluderDoesNotEraseHiddenVehicle) {
  Detector d; d.setBackground(background()); update(d,vehicle(),1.0);
  update(d,vehicle(.3),1.1);
  EXPECT_TRUE(d.occupied({1.2,0},.025,1.1));
  EXPECT_EQ(d.tracks().size(),2U);
}
TEST(PriorVehicleDetection, InvalidBackgroundCannotBecomeFreeSpace) {
  Detector d;
  EXPECT_THROW(d.setBackground({}), std::invalid_argument);
  Detector::Cloud invalid;
  invalid.emplace_back(NAN, NAN, NAN);
  EXPECT_THROW(d.setBackground(invalid), std::invalid_argument);
  EXPECT_FALSE(d.backgroundReady());
}

TEST(PriorVehicleDetection, LowVehicleBetweenFloorAndRoofIsNotBackground) {
  auto map = background();
  for (int x = -40; x <= 80; ++x) for (int y = -30; y <= 30; ++y)
    map.emplace_back(x * .05F, y * .05F, .25F);
  Detector d; d.setBackground(map);
  Cloud car;
  for (int y=-5;y<=5;++y) for(int z=2;z<=4;++z) {
    pcl::PointXYZI p; p.x=1.2F;p.y=y*.05F;p.z=z*.05F;car.push_back(p);
  }
  d.update(car,Eigen::Isometry3d::Identity(),{0,0,.20},{0,0,0},1.0);
  EXPECT_FALSE(d.tracks().empty());
  EXPECT_TRUE(d.occupied({1.3,0},.025,1.0));
}

TEST(PriorVehicleDetection, NoMotionExtrapolationDuringVisibilityGap) {
  Detector d; d.setBackground(background());
  update(d,vehicle(1.2),1.0);update(d,vehicle(1.3),1.2);
  ASSERT_EQ(d.tracks().size(),1U);
  EXPECT_FALSE(d.occupied({2.1,0},.025,1.2));
  ASSERT_FALSE(d.boxes(1.2).empty());
  const auto observed = d.boxes(1.2)[0];
  const auto retained = d.boxes(4.0)[0];
  EXPECT_LT(observed.maximum.x(), 2.1F);
  EXPECT_TRUE(observed.minimum.isApprox(retained.minimum));
  EXPECT_TRUE(observed.maximum.isApprox(retained.maximum));
  EXPECT_FALSE(d.occupied({2.1,0},.025,4.0));
  EXPECT_TRUE(d.occupied({1.3,0},.025,4.0));
}


TEST(PriorVehicleDetection, SparseFloorStripInSurveyGapIsStatic) {
  auto map = background();
  map.erase(std::remove_if(map.begin(), map.end(), [](const auto &p) {
    return p.z == 0.0F && p.x > 0.9F && p.x < 1.3F;
  }), map.end());
  Detector d; d.setBackground(map);
  Cloud strip;
  // 4 mm vertical span, wider than min_span; no coincident prior samples.
  for (int y=-8; y<=8; ++y) {
    pcl::PointXYZI p; p.x=1.10F; p.y=y*.05F; p.z=.035F+(y%2)*.002F;
    strip.push_back(p);
  }
  update(d, strip, 1.0);
  EXPECT_TRUE(d.tracks().empty());
  EXPECT_GT(d.stats().surface_filtered, 0U);
}

TEST(PriorVehicleDetection, ParallelCarRoofIsNotAbsorbedIntoFloor) {
  Detector d; d.setBackground(background());
  Cloud roof;
  for (int x=20; x<=28; ++x) for(int y=-4; y<=4; ++y) {
    pcl::PointXYZI p; p.x=x*.05F; p.y=y*.05F; p.z=.12F;
    roof.push_back(p);
  }
  update(d, roof, 1.0);
  ASSERT_FALSE(d.tracks().empty());
  EXPECT_TRUE(d.occupied({1.2,0},.025,1.0));
}

TEST(PriorVehicleDetection, OffsetRampAndTunnelEdgeStayStaticAcrossSensorRotation) {
  Detector::Cloud map;
  for (int x=-30; x<=60; ++x) for (int y=-20; y<=20; ++y) {
    map.emplace_back(x*.05F,y*.05F,.2F*x*.05F);
    map.emplace_back(x*.05F,y*.05F,.2F*x*.05F+.35F);
  }
  Detector d; d.setBackground(map);
  for (double yaw : {0.0, .7, 1.5, 3.0}) {
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
    pose.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Cloud observed;
    for (size_t i=0; i<map.size(); i+=7) {
      const auto &p=map[i];
      const Eigen::Vector3d v=pose.inverse()*Eigen::Vector3d(p.x+.025,p.y+.02,p.z+.025);
      pcl::PointXYZI q; q.x=v.x();q.y=v.y();q.z=v.z();observed.push_back(q);
    }
    d.update(observed,pose,{0,0,.25},{0,0,0},1.0+yaw);
    EXPECT_TRUE(d.tracks().empty());
  }
}

TEST(PriorVehicleDetection, SurfaceConfigRejectsInvalidBounds) {
  rog_map::PriorDynamicConfig config;
  config.static_surface_distance = NAN;
  EXPECT_THROW(config.validate(),std::invalid_argument);
  config=rog_map::PriorDynamicConfig{};
  config.static_surface_radius=.01;
  EXPECT_THROW(config.validate(),std::invalid_argument);
  config=rog_map::PriorDynamicConfig{};
  config.static_surface_min_support=33;
  EXPECT_THROW(config.validate(),std::invalid_argument);
}


TEST(PriorVehicleDetection, BoundaryFringeDoesNotSuppressCarInOpenWhiteSpace) {
  Detector d; d.setBackground(background());
  Cloud cloud = vehicle(.10F);
  const auto car = vehicle(1.2F);
  cloud += car;
  d.update(cloud,Eigen::Isometry3d::Identity(),{0,0,.5},{0,0,0},1.0,
      [](double x,double) {return x > .20;});
  ASSERT_EQ(d.tracks().size(),1U);
  EXPECT_GT(d.stats().boundary_filtered,0U);
  EXPECT_FALSE(d.occupied({.10,0},.025,1.0));
  EXPECT_TRUE(d.occupied({1.2,0},.025,1.0));
}

TEST(PriorVehicleDetection, StaticBoundaryDistanceUsesCellEdgesAndMapRotation) {
  rog_map::PriorMapData map;
  map.loaded=true;map.width=map.height=10;map.resolution=.1;
  map.origin_x=1.0;map.origin_y=2.0;
  map.occupied.assign(100,0U);map.known_free.assign(100,1U);
  map.occupied[5*10+4]=1U;
  EXPECT_FALSE(rog_map::priorMapKnownFree(map,1.45,2.45));
  EXPECT_TRUE(rog_map::priorMapKnownFree(map,1.55,2.45));
  map.known_free[5*10+5]=0U;
  EXPECT_FALSE(rog_map::priorMapKnownFree(map,1.55,2.45));
  EXPECT_FALSE(rog_map::priorMapKnownFree(map,20.0,20.0));
  EXPECT_FALSE(rog_map::priorMapKnownFree(map,NAN,2.45));
  EXPECT_TRUE(rog_map::priorMapNearOccupied(map,1.45,2.45,0.0));
  EXPECT_FALSE(rog_map::priorMapNearOccupied(map,1.55,2.45,.049));
  EXPECT_TRUE(rog_map::priorMapNearOccupied(map,1.55,2.45,.051));
  EXPECT_FALSE(rog_map::priorMapNearOccupied(map,1.55,2.55,.06));
  EXPECT_TRUE(rog_map::priorMapNearOccupied(map,1.55,2.55,.071));
  map.origin_yaw=std::acos(-1.0)/2.0;
  EXPECT_TRUE(rog_map::priorMapNearOccupied(map,.55,2.55,.051));
  EXPECT_FALSE(rog_map::priorMapNearOccupied(map,.55,2.55,.049));
  EXPECT_FALSE(rog_map::priorMapNearOccupied(map,NAN,2.55,.1));
  EXPECT_FALSE(rog_map::priorMapNearOccupied(map,.55,2.55,-.1));
  EXPECT_FALSE(rog_map::priorMapNearOccupied(map,1e20,1e20,.1));
}


TEST(PriorVehicleDetection, PassageProfileAbsorbsSevenCentimeterStaticResidual) {
  Detector d; d.setBackground(background());
  Cloud strip;
  for (int y=-7; y<=7; ++y) {
    pcl::PointXYZI p;p.x=1.2F;p.y=y*.05F;p.z=.07F+(y%2)*.002F;strip.push_back(p);
  }
  update(d,strip,1.0);
  EXPECT_TRUE(d.tracks().empty());
}


TEST(PriorVehicleDetection, SurveyedRampFaceIsStaticButCarAboveRampStillBlocks) {
  Detector d; d.setBackground(background());
  Cloud face;
  for(int x=15;x<=30;++x) for(int z=2;z<=4;++z){
    pcl::PointXYZI p;p.x=x*.05F;p.y=0;p.z=z*.05F;face.push_back(p);
  }
  const auto ground=[](double,double,double &z){z=.209;return true;};
  d.update(face,Eigen::Isometry3d::Identity(),{0,0,.5},{0,0,.209},1.0,{},ground);
  EXPECT_TRUE(d.tracks().empty());
  EXPECT_GT(d.stats().ground_filtered,0U);
  Cloud roof;
  for(int x=20;x<=28;++x)for(int y=-4;y<=4;++y){
    pcl::PointXYZI p;p.x=x*.05F;p.y=y*.05F;p.z=.329F;roof.push_back(p);
  }
  d.update(roof,Eigen::Isometry3d::Identity(),{0,0,.5},{0,0,.209},1.1,{},ground);
  EXPECT_TRUE(d.occupied({1.2,0},.025,1.1));
}

TEST(PriorVehicleDetection, MissingSurveyedGroundDoesNotSuppressLiveVehicle) {
  Detector d; d.setBackground(background());
  d.update(vehicle(),Eigen::Isometry3d::Identity(),{0,0,.5},{0,0,0},1.0,{},
      [](double,double,double &z){z=NAN;return false;});
  EXPECT_TRUE(d.occupied({1.2,0},.025,1.0));
}


TEST(PriorVehicleDetection, BodyCompletionKeepsMeasuredFaceAndTangentialCenter) {
  Detector d;d.setBackground(background());update(d,vehicle(1.2),1.0);
  ASSERT_EQ(d.tracks().size(),1U);
  EXPECT_NEAR(d.tracks()[0].minimum.x(),1.2,1e-5);
  EXPECT_NEAR(d.tracks()[0].maximum.x(),1.2,1e-5);
  EXPECT_TRUE(d.occupied({1.15,0},.025,1.0));
  EXPECT_NEAR(d.tracks()[0].completion_shift.y(),0.0,1e-6);
  EXPECT_TRUE(d.occupied({1.75,0},.025,1.0));
}

TEST(PriorVehicleDetection, NarrowFreeRayStripCannotClearWholeVehicle) {
  Detector d;d.setBackground(background());update(d,vehicle(),1.0);
  Cloud rays;
  for(int z=6;z<=14;++z) {
    pcl::PointXYZI p;p.x=3.0F;p.y=0.0F;p.z=z*.05F;rays.push_back(p);
  }
  update(d,rays,1.1);update(d,rays,1.2);
  EXPECT_EQ(d.tracks().size(),1U);
  EXPECT_TRUE(d.occupied({1.2,0},.025,1.2));
  EXPECT_EQ(d.stats().ray_cleared,0U);
}


TEST(PriorVehicleDetection, ThinRoofPatchIsNotShiftedAsAFrontFace) {
  Detector d;d.setBackground(background());Cloud roof;
  for(int x=24;x<=27;++x)for(int y=-4;y<=4;++y){
    pcl::PointXYZI p;p.x=x*.05F;p.y=y*.05F;p.z=.24F;roof.push_back(p);
  }
  update(d,roof,1.0);
  ASSERT_EQ(d.tracks().size(),1U);
  EXPECT_NEAR(d.tracks()[0].completion_shift.norm(),0.0,1e-6);
}

TEST(PriorVehicleDetection, DiagonalViewDoesNotShiftAlongVerticalFace) {
  Detector d;d.setBackground(background());Cloud face;
  for(int y=2;y<=5;++y)for(int z=5;z<=12;++z){
    pcl::PointXYZI p;p.x=1.2F;p.y=y*.05F;p.z=z*.05F;face.push_back(p);
  }
  d.update(face,Eigen::Isometry3d::Identity(),{0,1.5,.5},{0,0,0},1.0);
  ASSERT_EQ(d.tracks().size(),1U);
  EXPECT_GT(d.tracks()[0].completion_shift.x(),0.0);
  EXPECT_NEAR(d.tracks()[0].completion_shift.y(),0.0,1e-6);
}


TEST(PriorVehicleDetection, BroadFasciaWithTopDoesNotGrowIntoAdjacentPassage) {
  Detector d;d.setBackground(background());Cloud fascia;
  for(int x=-10;x<=10;++x) {
    for(int z=2;z<=4;++z){
      pcl::PointXYZI p;p.x=x*.05F;p.y=1.2F;p.z=z*.05F;fascia.push_back(p);
    }
    for(int y=24;y<=28;++y){
      pcl::PointXYZI p;p.x=x*.05F;p.y=y*.05F;p.z=.20F;fascia.push_back(p);
    }
  }
  d.update(fascia,Eigen::Isometry3d::Identity(),{0,0,.3},{0,0,0},1.0);
  ASSERT_FALSE(d.tracks().empty());
  EXPECT_FALSE(d.occupied({0,.99},.025,1.0));
  EXPECT_TRUE(d.occupied({0,1.2},.025,1.0));
  EXPECT_GT(d.tracks()[0].completion_shift.y(),0.0);
}

} // namespace
