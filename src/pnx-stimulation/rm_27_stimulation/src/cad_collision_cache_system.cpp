// 为已生成的凸包预注册精确网格，避免 Bullet 再次体素分解造成 CAD 漏包。
#include <array>
#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace rm_27_stimulation {
class CadCollisionCacheSystem : public gz::sim::System,
                                public gz::sim::ISystemConfigure {
public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override {
    const auto directory = _sdf->Get<std::string>("mesh_directory");
    std::set<std::string> names;
    // 与实际 collision 共用生成清单。安装目录可能残留旧版 STL，不能扫描后
    // 把它们也预注册，更不能让残留文件占用当前凸体预算。
    if (!_sdf->HasElement("mesh_file"))
      throw std::runtime_error("CAD collision mesh_file manifest is missing");
    for (auto element = _sdf->FindElement("mesh_file"); element;
         element = element->GetNextElement("mesh_file")) {
      const std::filesystem::path file(element->Get<std::string>());
      if (file != file.filename() || file.extension() != ".stl" ||
          !names.insert(file.stem().string()).second)
        throw std::runtime_error("CAD collision mesh_file is invalid: " +
                                 file.string());
    }
    if (names.empty() || names.size() > 64u)
      throw std::runtime_error("CAD collision mesh count outside budget: " +
                               directory);
    auto &manager = *gz::common::MeshManager::Instance();
    for (const auto &name : names) {
      const auto path =
          (std::filesystem::path(directory) / (name + ".stl")).string();
      const auto *source = manager.Load(path);
      if (!source)
        throw std::runtime_error("CAD collision mesh cannot load: " + path);
      // gz-physics7 的 convex_hull 分支首先查询这个缓存名。
      // 这里只缓存生成工具已经验证过的凸包；不修改赛场网格或其他模型。
      const std::array<std::string, 3> aliases = {
          source->Name(),
          "model://rm_27_stimulation/meshes/rm27_sentry_collision/" + name +
              ".stl",
          "package://rm_27_stimulation/meshes/rm27_sentry_collision/" + name +
              ".stl"};
      for (const auto &alias : aliases) {
        const auto cacheName = alias + "__CONVEX_1_200000";
        if (manager.MeshByName(cacheName))
          continue;
        auto exact = gz::common::MeshManager::MergeSubMeshes(*source);
        if (!exact || exact->SubMeshCount() != 1u)
          throw std::runtime_error(
              "CAD collision mesh has invalid submeshes: " + path);
        exact->SetName(cacheName);
        manager.AddMesh(exact.release());
      }
    }
    gzmsg << "[CadCollision] registered " << names.size()
          << " exact CAD convex meshes; "
          << "Bullet margin=1 mm, no voxel simplification." << std::endl;
  }
};
} // namespace rm_27_stimulation

GZ_ADD_PLUGIN(rm_27_stimulation::CadCollisionCacheSystem, gz::sim::System,
              rm_27_stimulation::CadCollisionCacheSystem::ISystemConfigure)
