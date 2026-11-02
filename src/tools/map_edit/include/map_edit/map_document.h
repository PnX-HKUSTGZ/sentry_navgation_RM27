#ifndef MAP_EDIT_MAP_DOCUMENT_H
#define MAP_EDIT_MAP_DOCUMENT_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <string>

namespace map_edit
{
// Keep the original YAML document, including extensions not interpreted by RViz.
class MapDocument
{
public:
  nav_msgs::msg::OccupancyGrid load(const std::string &yaml_file);
  // Always Save As: the loaded YAML, image and elevation asset are protected.
  // Returns the image filename; throws on incompatible encoding or I/O failure.
  std::string save(const std::string &yaml_file,
                   const nav_msgs::msg::OccupancyGrid &map) const;
  const YAML::Node &metadata() const { return metadata_; }
private:
  YAML::Node metadata_;
  std::string source_yaml_;
  std::string source_image_;
};
}
#endif
