// 为已生成的凸包预注册精确网格，避免 Bullet 再次体素分解造成 CAD 漏包。
#include <array>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace rm_27_stimulation
{
class CadCollisionCacheSystem : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const auto directory = _sdf->Get<std::string>("mesh_directory");
    std::vector<std::string> names = {"chassis_body", "chassis_armor", "yaw_body",
      "yaw_sensors", "roll_body", "wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    // 未指定时保留机器人原有九个部件；场地可显式指定已经验证为凸形的局部件。
    if (_sdf->HasElement("mesh_names"))
    {
      names.clear();
      std::istringstream input(_sdf->Get<std::string>("mesh_names"));
      for (std::string name; input >> name; )
        names.push_back(name);
      if (names.empty())
        throw std::runtime_error("mesh_names must not be empty");
    }
    const auto uriPrefix = _sdf->HasElement("uri_prefix")
      ? _sdf->Get<std::string>("uri_prefix")
      : std::string("model://rm_27_stimulation/meshes/rm27_sentry_collision/");
    auto &manager = *gz::common::MeshManager::Instance();
    for (const auto &name : names)
    {
      const auto path = (std::filesystem::path(directory) / (name + ".stl")).string();
      const auto *source = manager.Load(path);
      if (!source)
        throw std::runtime_error("CAD collision mesh cannot load: " + path);
      // gz-physics7 的 convex_hull 分支首先查询这个缓存名。
      // 仅缓存指定的凸网格，不进行二次体素化。
      std::vector<std::string> aliases = {source->Name(), uriPrefix + name + ".stl"};
      if (!_sdf->HasElement("uri_prefix"))
        aliases.push_back("package://rm_27_stimulation/meshes/rm27_sentry_collision/" +
          name + ".stl");
      for (const auto &alias : aliases)
      {
        const auto cacheName = alias + "__CONVEX_1_200000";
        if (manager.MeshByName(cacheName))
          continue;
        auto exact = gz::common::MeshManager::MergeSubMeshes(*source);
        if (!exact || exact->SubMeshCount() != 1u)
          throw std::runtime_error("CAD collision mesh has invalid submeshes: " + path);
        exact->SetName(cacheName);
        manager.AddMesh(exact.release());
      }
    }
    gzmsg << "[CadCollision] registered " << names.size()
          << " exact CAD convex meshes from " << directory
          << "; Bullet margin=1 mm, no voxel simplification." << std::endl;
  }
};
}  // namespace rm_27_stimulation

GZ_ADD_PLUGIN(rm_27_stimulation::CadCollisionCacheSystem, gz::sim::System,
  rm_27_stimulation::CadCollisionCacheSystem::ISystemConfigure)
