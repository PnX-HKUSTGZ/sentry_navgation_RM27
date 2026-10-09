#include "map_edit/map_document.h"
#include <nav2_map_server/map_io.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <QImage>
#include <QSaveFile>
#include <QTemporaryDir>
#include <QFile>
#include <filesystem>
#include <array>
#include <stdexcept>
#include <functional>

namespace map_edit
{
namespace
{
namespace fs = std::filesystem;
fs::path absolutePath(const fs::path &path)
{
  return fs::weakly_canonical(fs::absolute(path));
}
void writeAtomic(const fs::path &destination, const QByteArray &bytes)
{
  QSaveFile file(QString::fromStdString(destination.string()));
  if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
    throw std::runtime_error("Cannot write " + destination.string());
}
}

nav_msgs::msg::OccupancyGrid MapDocument::load(const std::string &yaml_file)
{
  const auto path = absolutePath(yaml_file);
  auto metadata = YAML::LoadFile(path.string());
  const auto parameters = nav2_map_server::loadMapYaml(path.string());
  nav_msgs::msg::OccupancyGrid map;
  nav2_map_server::loadMapFromFile(parameters, map);
  if (map.data.empty()) throw std::runtime_error("Map has no cells");
  metadata_ = YAML::Clone(metadata);
  source_yaml_ = path.string();
  source_image_ = absolutePath(parameters.image_file_name).string();
  return map;
}

std::string MapDocument::save(const std::string &yaml_file,
                              const nav_msgs::msg::OccupancyGrid &map) const
{
  if (map.info.width == 0 || map.info.height == 0 ||
      map.data.size() != static_cast<size_t>(map.info.width) * map.info.height)
    throw std::runtime_error("Invalid map dimensions");
  const auto destination = absolutePath(yaml_file);
  auto output = metadata_.IsMap() ? YAML::Clone(metadata_) : YAML::Load(
    "mode: trinary\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n");
  const std::string mode = output["mode"] ? output["mode"].as<std::string>() : "trinary";
  const auto image_path = destination.parent_path() /
    (destination.stem().string() + (mode == "scale" ? ".png" : ".pgm"));
  if ((!source_yaml_.empty() && destination == absolutePath(source_yaml_)) ||
      (!source_image_.empty() && (image_path == absolutePath(source_image_) ||
                                 destination == absolutePath(source_image_))))
    throw std::runtime_error("Save As requires a new path; the source map is protected");
  if (fs::exists(destination) || fs::exists(image_path))
    throw std::runtime_error("Save As requires new YAML and image paths; existing files are protected");
  auto rebase_elevation = [&](YAML::Node reference) {
    const fs::path elevation(reference.as<std::string>());
    if (elevation.empty()) return;
    const auto target = elevation.is_absolute() ? absolutePath(elevation) :
      absolutePath(fs::path(source_yaml_).parent_path() / elevation);
    if (destination == target || image_path == target)
      throw std::runtime_error("The elevation asset is protected");
    if (elevation.is_relative())
      reference = target.lexically_relative(destination.parent_path()).string();
  };
  std::function<void(YAML::Node)> rebase_grid = [&](YAML::Node node) {
    if (node.IsMap()) {
      for (auto entry : node) {
        if (entry.first.as<std::string>() == "image" && entry.second.IsScalar())
          rebase_elevation(entry.second);
        else rebase_grid(entry.second);
      }
    } else if (node.IsSequence()) {
      for (auto entry : node) rebase_grid(entry);
    }
  };
  if (output["ground_elevation"]) {
    if (output["ground_elevation"].IsScalar()) rebase_elevation(output["ground_elevation"]);
    else rebase_grid(output["ground_elevation"]);
  }
  output["image"] = image_path.filename().string();
  // Geometry is retained exactly when it has not changed (avoid float resolution rounding).
  if (!output["resolution"]) output["resolution"] = map.info.resolution;
  if (!output["origin"]) {
    output["origin"].push_back(map.info.origin.position.x);
    output["origin"].push_back(map.info.origin.position.y);
    output["origin"].push_back(tf2::getYaw(map.info.origin.orientation));
  }

  QTemporaryDir staging;
  if (!staging.isValid()) throw std::runtime_error("Cannot create temporary directory");
  const fs::path stage(staging.path().toStdString());
  const auto staged_yaml = stage / destination.filename();
  const auto staged_image = stage / image_path.filename();
  const auto yaml_text = QByteArray::fromStdString(YAML::Dump(output) + "\n");
  writeAtomic(staged_yaml, yaml_text);
  auto parameters = nav2_map_server::loadMapYaml(staged_yaml.string());

  // Ask the installed Nav2 reader how every grayscale byte is interpreted. This
  // handles version-specific boundary/rounding rules and both negate conventions.
  QImage palette(256, 1, QImage::Format_Grayscale8);
  for (int i = 0; i < 256; ++i) palette.scanLine(0)[i] = static_cast<uchar>(i);
  const auto palette_path = stage / "palette.pgm";
  if (!palette.save(QString::fromStdString(palette_path.string()), "PGM"))
    throw std::runtime_error("Cannot encode palette");
  auto palette_parameters = parameters;
  palette_parameters.image_file_name = palette_path.string();
  nav_msgs::msg::OccupancyGrid decoded_palette;
  nav2_map_server::loadMapFromFile(palette_parameters, decoded_palette);
  std::array<int, 102> encoding;
  encoding.fill(-1);
  for (int i = 0; i < 256; ++i) {
    const int value = decoded_palette.data.at(i);
    if (value >= -1 && value <= 100 && encoding[value + 1] < 0) encoding[value + 1] = i;
  }

  const bool alpha = mode == "scale";
  QImage image(map.info.width, map.info.height,
               alpha ? QImage::Format_ARGB32 : QImage::Format_Grayscale8);
  for (uint32_t y = 0; y < map.info.height; ++y) {
    for (uint32_t x = 0; x < map.info.width; ++x) {
      const int value = map.data[(map.info.height - 1 - y) * map.info.width + x];
      if (alpha && value == -1) { image.setPixel(x, y, qRgba(0, 0, 0, 0)); continue; }
      if (value < -1 || value > 100 || encoding[value + 1] < 0)
        throw std::runtime_error("Map value cannot be represented without changing mode/thresholds");
      const auto pixel = static_cast<uchar>(encoding[value + 1]);
      if (alpha) image.setPixel(x, y, qRgba(pixel, pixel, pixel, 255));
      else image.scanLine(y)[x] = pixel;
    }
  }
  if (!image.save(QString::fromStdString(staged_image.string()), alpha ? "PNG" : "PGM"))
    throw std::runtime_error("Cannot encode map image");
  nav_msgs::msg::OccupancyGrid verified;
  nav2_map_server::loadMapFromFile(parameters, verified);
  if (verified.data != map.data || verified.info.width != map.info.width ||
      verified.info.height != map.info.height)
    throw std::runtime_error("Saved map failed exact Nav2 occupancy round-trip validation");
  QFile image_file(QString::fromStdString(staged_image.string()));
  if (!image_file.open(QIODevice::ReadOnly)) throw std::runtime_error("Cannot read staged image");
  writeAtomic(image_path, image_file.readAll());
  writeAtomic(destination, yaml_text);
  return image_path.string();
}
}
