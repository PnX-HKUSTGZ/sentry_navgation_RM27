#include <rog_map/prior_map.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace rog_map {

namespace {

std::string readPgmToken(std::istream &input) {
  while (input) {
    const int next = input.peek();
    if (next == std::char_traits<char>::eof()) {
      return {};
    }
    if (std::isspace(static_cast<unsigned char>(next))) {
      input.get();
      continue;
    }
    if (next == '#') {
      input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    break;
  }

  std::string token;
  while (input) {
    const int next = input.peek();
    if (next == std::char_traits<char>::eof() ||
        std::isspace(static_cast<unsigned char>(next)) || next == '#') {
      break;
    }
    token.push_back(static_cast<char>(input.get()));
  }
  return token;
}

int parsePgmInteger(const std::string &token, const std::string &field,
                    const std::string &path) {
  if (token.empty()) {
    throw std::runtime_error("[PriorMap] missing PGM " + field + " in '" +
                             path + "'");
  }
  size_t parsed = 0;
  int value = 0;
  try {
    value = std::stoi(token, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error("[PriorMap] invalid PGM " + field + " '" + token +
                             "' in '" + path + "'");
  }
  if (parsed != token.size()) {
    throw std::runtime_error("[PriorMap] invalid PGM " + field + " '" + token +
                             "' in '" + path + "'");
  }
  return value;
}

struct RawPgm {
  int width{0};
  int height{0};
  int max_value{0};
  std::vector<uint16_t> pixels;
};

RawPgm loadRawPgm(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    throw std::runtime_error("[PriorMap] cannot open PGM file '" + path + "'");
  }

  const std::string magic = readPgmToken(input);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("[PriorMap] unsupported PGM format '" + magic +
                             "' in '" + path + "' (expected P5 or P2)");
  }
  RawPgm raster;
  raster.width = parsePgmInteger(readPgmToken(input), "width", path);
  raster.height = parsePgmInteger(readPgmToken(input), "height", path);
  raster.max_value =
      parsePgmInteger(readPgmToken(input), "maximum gray value", path);
  if (raster.width <= 0 || raster.height <= 0) {
    throw std::runtime_error("[PriorMap] PGM dimensions must be positive in '" +
                             path + "'");
  }
  if (raster.max_value <= 0 || raster.max_value > 65535) {
    throw std::runtime_error(
        "[PriorMap] PGM maximum gray value must be in [1, 65535] in '" + path +
        "'");
  }
  const size_t pixel_count =
      static_cast<size_t>(raster.width) * static_cast<size_t>(raster.height);
  if (pixel_count / static_cast<size_t>(raster.width) !=
      static_cast<size_t>(raster.height)) {
    throw std::runtime_error(
        "[PriorMap] PGM dimensions overflow pixel count in '" + path + "'");
  }

  raster.pixels.assign(pixel_count, 0U);

  if (magic == "P5") {
    const int separator = input.get();
    if (separator == std::char_traits<char>::eof() ||
        !std::isspace(static_cast<unsigned char>(separator))) {
      throw std::runtime_error("[PriorMap] missing PGM raster separator in '" +
                               path + "'");
    }
    if (separator == '\r' && input.peek() == '\n') {
      input.get();
    }
    const size_t bytes_per_pixel = raster.max_value < 256 ? 1U : 2U;
    if (pixel_count >
        std::numeric_limits<size_t>::max() / bytes_per_pixel) {
      throw std::runtime_error(
          "[PriorMap] PGM raster byte count overflows in '" + path + "'");
    }
    const size_t byte_count = pixel_count * bytes_per_pixel;
    std::vector<unsigned char> raw(byte_count, 0U);
    input.read(reinterpret_cast<char *>(raw.data()),
               static_cast<std::streamsize>(byte_count));
    if (input.gcount() != static_cast<std::streamsize>(byte_count)) {
      throw std::runtime_error(
          "[PriorMap] PGM pixel count is smaller than declared in '" + path +
          "'");
    }
    if (input.peek() != std::char_traits<char>::eof()) {
      throw std::runtime_error(
          "[PriorMap] PGM pixel count is larger than declared in '" + path +
          "'");
    }
    for (size_t i = 0; i < pixel_count; ++i) {
      const uint16_t value = bytes_per_pixel == 1U
                                 ? static_cast<uint16_t>(raw[i])
                                 : static_cast<uint16_t>(
                                       (static_cast<uint16_t>(raw[2U * i])
                                        << 8U) |
                                       static_cast<uint16_t>(raw[2U * i + 1U]));
      if (value > raster.max_value) {
        throw std::runtime_error(
            "[PriorMap] PGM pixel exceeds maximum gray value in '" + path +
            "'");
      }
      raster.pixels[i] = value;
    }
  } else {
    for (size_t i = 0; i < pixel_count; ++i) {
      const int value = parsePgmInteger(readPgmToken(input), "pixel", path);
      if (value < 0 || value > raster.max_value) {
        throw std::runtime_error(
            "[PriorMap] PGM pixel is outside the declared gray range in '" +
            path + "'");
      }
      raster.pixels[i] = static_cast<uint16_t>(value);
    }
    if (!readPgmToken(input).empty()) {
      throw std::runtime_error(
          "[PriorMap] PGM pixel count is larger than declared in '" + path +
          "'");
    }
  }
  return raster;
}

std::vector<uint8_t> loadPgm(const std::string &path, int &width, int &height) {
  const RawPgm raster = loadRawPgm(path);
  if (raster.max_value > 255) {
    throw std::runtime_error(
        "[PriorMap] occupancy PGM maximum gray value must be in [1, 255] in '" +
        path + "'");
  }
  width = raster.width;
  height = raster.height;
  std::vector<uint8_t> pixels(raster.pixels.size(), 0U);
  for (size_t i = 0; i < raster.pixels.size(); ++i) {
    pixels[i] = static_cast<uint8_t>(std::lround(
        static_cast<double>(raster.pixels[i]) * 255.0 /
        static_cast<double>(raster.max_value)));
  }
  return pixels;
}

double requireFiniteScalar(const YAML::Node &root, const char *key,
                           const std::string &path) {
  if (!root[key] || !root[key].IsScalar()) {
    throw std::runtime_error("[PriorMap] required YAML scalar '" +
                             std::string(key) + "' is missing in '" + path +
                             "'");
  }
  const double value = root[key].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error("[PriorMap] YAML scalar '" + std::string(key) +
                             "' must be finite in '" + path + "'");
  }
  return value;
}

template <size_t Size>
std::array<double, Size> requireFiniteSequence(const YAML::Node &root,
                                               const char *key,
                                               const std::string &path) {
  const YAML::Node sequence = root[key];
  if (!sequence || !sequence.IsSequence() || sequence.size() != Size) {
    throw std::runtime_error("[PriorMap] YAML '" + std::string(key) +
                             "' must contain " + std::to_string(Size) +
                             " values in '" + path + "'");
  }
  std::array<double, Size> values{};
  for (size_t index = 0; index < Size; ++index) {
    values[index] = sequence[index].as<double>();
    if (!std::isfinite(values[index])) {
      throw std::runtime_error("[PriorMap] YAML '" + std::string(key) +
                               "' values must be finite in '" + path + "'");
    }
  }
  return values;
}

size_t checkedCellCount(int width, int height) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument(
        "PriorMap projection dimensions must be positive");
  }
  const size_t width_size = static_cast<size_t>(width);
  const size_t height_size = static_cast<size_t>(height);
  if (width_size > std::numeric_limits<size_t>::max() / height_size) {
    throw std::invalid_argument(
        "PriorMap projection dimensions overflow cell count");
  }
  return width_size * height_size;
}

bool priorStorageValid(const PriorMapData &prior_map) {
  if (!prior_map.loaded || prior_map.width <= 0 || prior_map.height <= 0 ||
      !std::isfinite(prior_map.resolution) || prior_map.resolution <= 0.0) {
    return false;
  }
  const size_t width = static_cast<size_t>(prior_map.width);
  const size_t height = static_cast<size_t>(prior_map.height);
  return width <= std::numeric_limits<size_t>::max() / height &&
         prior_map.occupied.size() == width * height &&
         prior_map.known_free.size() == width * height;
}

bool groundElevationAtMapPoint(const PriorMapData &prior_map, double map_x,
                               double map_y, double &support_z_map) {
  if (!prior_map.ground_elevation_loaded || !std::isfinite(map_x) ||
      !std::isfinite(map_y)) {
    return false;
  }
  bool support_known = true;
  support_z_map = prior_map.ground_elevation_default_height;
  if (prior_map.ground_elevation_grid_loaded) {
    const double dx = map_x - prior_map.origin_x;
    const double dy = map_y - prior_map.origin_y;
    const double local_x = prior_map.fast_origin_yaw_cos * dx +
                           prior_map.fast_origin_yaw_sin * dy;
    const double local_y = -prior_map.fast_origin_yaw_sin * dx +
                           prior_map.fast_origin_yaw_cos * dy;
    const int image_col =
        static_cast<int>(std::floor(local_x / prior_map.resolution));
    const int map_row =
        static_cast<int>(std::floor(local_y / prior_map.resolution));
    support_known = image_col >= 0 && image_col < prior_map.width &&
                    map_row >= 0 && map_row < prior_map.height;
    if (support_known) {
      const int image_row = prior_map.height - 1 - map_row;
      const size_t index = static_cast<size_t>(image_row) *
                               static_cast<size_t>(prior_map.width) +
                           static_cast<size_t>(image_col);
      support_known = index < prior_map.ground_elevation_grid.size() &&
                      prior_map.ground_elevation_grid[index] !=
                          prior_map.ground_elevation_grid_no_data;
      if (support_known) {
        support_z_map = prior_map.ground_elevation_grid_offset +
                        prior_map.ground_elevation_grid_scale *
                            static_cast<double>(
                                prior_map.ground_elevation_grid[index]);
      }
    }
  }
  for (const auto &patch : prior_map.ground_elevation_patches) {
    if (map_x + 1.0e-9 < patch.min_x || map_x - 1.0e-9 > patch.max_x ||
        map_y + 1.0e-9 < patch.min_y || map_y - 1.0e-9 > patch.max_y) {
      continue;
    }
    support_z_map =
        patch.reference_z + patch.slope_x * (map_x - patch.reference_x) +
        patch.slope_y * (map_y - patch.reference_y);
    support_known = true;
  }
  return support_known && std::isfinite(support_z_map);
}

uint8_t priorMapStateWithRotation(const PriorMapData &prior_map, double map_x,
                                  double map_y, double origin_yaw_cos,
                                  double origin_yaw_sin) {
  if (!priorStorageValid(prior_map) || !std::isfinite(map_x) ||
      !std::isfinite(map_y)) {
    return 0U;
  }

  const double dx = map_x - prior_map.origin_x;
  const double dy = map_y - prior_map.origin_y;
  const double local_x = origin_yaw_cos * dx + origin_yaw_sin * dy;
  const double local_y = -origin_yaw_sin * dx + origin_yaw_cos * dy;
  const double image_col_value = local_x / prior_map.resolution;
  const double map_row_value = local_y / prior_map.resolution;
  if (!std::isfinite(image_col_value) || !std::isfinite(map_row_value) ||
      image_col_value < 0.0 ||
      image_col_value >= static_cast<double>(prior_map.width) ||
      map_row_value < 0.0 ||
      map_row_value >= static_cast<double>(prior_map.height)) {
    return 0U;
  }

  const int image_col = static_cast<int>(std::floor(image_col_value));
  const int image_row =
      prior_map.height - 1 - static_cast<int>(std::floor(map_row_value));
  const size_t index =
      static_cast<size_t>(image_row) * static_cast<size_t>(prior_map.width) +
      static_cast<size_t>(image_col);
  if (prior_map.occupied[index] != 0U) {
    return 2U;
  }
  return prior_map.known_free[index] != 0U ? 1U : 0U;
}

uint8_t samplePriorAtRogPointFast(const PriorMapData &prior_map, double rog_x,
                                  double rog_y) {
  if (!prior_map.transform_ready) {
    return 0U;
  }
  const double map_x = prior_map.fixed_transform_cos * rog_x -
                       prior_map.fixed_transform_sin * rog_y +
                       prior_map.fixed_transform.tx;
  const double map_y = prior_map.fixed_transform_sin * rog_x +
                       prior_map.fixed_transform_cos * rog_y +
                       prior_map.fixed_transform.ty;
  return priorMapStateWithRotation(prior_map, map_x, map_y,
                                   prior_map.fast_origin_yaw_cos,
                                   prior_map.fast_origin_yaw_sin);
}

uint8_t samplePriorCellConservative(const PriorMapData &prior_map, double rog_x,
                                    double rog_y, double rog_resolution) {
  const double half_extent = 0.5 * rog_resolution * (1.0 - 1.0e-6);
  bool all_known_free = true;
  for (const double dy : {-half_extent, 0.0, half_extent}) {
    for (const double dx : {-half_extent, 0.0, half_extent}) {
      const uint8_t state =
          samplePriorAtRogPointFast(prior_map, rog_x + dx, rog_y + dy);
      if (state == 2U) {
        return 2U;
      }
      all_known_free = all_known_free && state == 1U;
    }
  }
  return all_known_free ? 1U : 0U;
}

bool sameProjectionGeometry(const PriorMapData &prior_map, int min_global_x,
                            int min_global_y, int width, int height,
                            double resolution, size_t expected_size) {
  return prior_map.cached_min_x == min_global_x &&
         prior_map.cached_min_y == min_global_y &&
         prior_map.cached_width == width && prior_map.cached_height == height &&
         std::abs(prior_map.cached_resolution - resolution) <= 1.0e-9 &&
         prior_map.cached_mask.size() == expected_size &&
         prior_map.cached_free_mask.size() == expected_size &&
         prior_map.cached_ground_support_mask.size() == expected_size &&
         prior_map.cached_ground_support_z.size() == expected_size;
}

void saveProjectionGeometry(PriorMapData &prior_map, int min_global_x,
                            int min_global_y, int width, int height,
                            double resolution) {
  prior_map.cached_min_x = min_global_x;
  prior_map.cached_min_y = min_global_y;
  prior_map.cached_width = width;
  prior_map.cached_height = height;
  prior_map.cached_resolution = resolution;
}

void rebuildPriorProjection(PriorMapData &prior_map, int min_global_x,
                            int min_global_y, int width, int height,
                            double resolution, double origin_x, double origin_y,
                            size_t expected_size) {
  prior_map.cached_mask.assign(expected_size, 1U);
  prior_map.cached_free_mask.assign(expected_size, 0U);
  prior_map.cached_ground_support_mask.assign(expected_size, 0U);
  prior_map.cached_ground_support_z.assign(
      expected_size, std::numeric_limits<float>::quiet_NaN());
  prior_map.scratch_mask.resize(expected_size);
  prior_map.scratch_free_mask.resize(expected_size);
  prior_map.scratch_ground_support_mask.resize(expected_size);
  prior_map.scratch_ground_support_z.resize(expected_size);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double rog_x =
          origin_x + (static_cast<double>(x) + 0.5) * resolution;
      const double rog_y =
          origin_y + (static_cast<double>(y) + 0.5) * resolution;
      const size_t index = static_cast<size_t>(y) * static_cast<size_t>(width) +
                           static_cast<size_t>(x);
      const uint8_t prior_state =
          samplePriorCellConservative(prior_map, rog_x, rog_y, resolution);
      if (prior_state == 2U) {
        prior_map.cached_mask[index] = 0U;
      } else if (prior_state == 1U) {
        prior_map.cached_free_mask[index] = 1U;
        double support_z_rog = 0.0;
        if (priorMapGroundSupport(prior_map, rog_x, rog_y, support_z_rog)) {
          prior_map.cached_ground_support_mask[index] = 1U;
          prior_map.cached_ground_support_z[index] =
              static_cast<float>(support_z_rog);
        }
      }
    }
  }
  saveProjectionGeometry(prior_map, min_global_x, min_global_y, width, height,
                         resolution);
  prior_map.projection_cache_ready = true;
}

} // namespace

PriorMapData loadPriorMap(const std::string &yaml_path,
                          const std::string &pgm_path) {
  if (yaml_path.empty()) {
    throw std::runtime_error(
        "[PriorMap] projection.prior_map.yaml_path must not be empty");
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception &error) {
    throw std::runtime_error("[PriorMap] failed to load YAML '" + yaml_path +
                             "': " + error.what());
  }

  PriorMapData prior;
  bool ground_elevation_grid_requested = false;
  std::filesystem::path ground_elevation_grid_path;
  try {
    if (root["mode"] && (!root["mode"].IsScalar() ||
                         root["mode"].as<std::string>() != "trinary")) {
      throw std::runtime_error(
          "[PriorMap] only YAML mode 'trinary' is supported in '" + yaml_path +
          "'");
    }
    prior.resolution = requireFiniteScalar(root, "resolution", yaml_path);
    prior.occupied_thresh =
        requireFiniteScalar(root, "occupied_thresh", yaml_path);
    prior.free_thresh = requireFiniteScalar(root, "free_thresh", yaml_path);
    if (!root["origin"] || !root["origin"].IsSequence() ||
        root["origin"].size() != 3U) {
      throw std::runtime_error(
          "[PriorMap] YAML 'origin' must contain [x, y, yaw] in '" + yaml_path +
          "'");
    }
    prior.origin_x = root["origin"][0].as<double>();
    prior.origin_y = root["origin"][1].as<double>();
    prior.origin_yaw = root["origin"][2].as<double>();
    if (!std::isfinite(prior.origin_x) || !std::isfinite(prior.origin_y) ||
        !std::isfinite(prior.origin_yaw)) {
      throw std::runtime_error(
          "[PriorMap] YAML origin values must be finite in '" + yaml_path +
          "'");
    }
    if (!root["negate"] || !root["negate"].IsScalar()) {
      throw std::runtime_error(
          "[PriorMap] required YAML scalar 'negate' is missing in '" +
          yaml_path + "'");
    }
    const int negate = root["negate"].as<int>();
    if (negate != 0 && negate != 1) {
      throw std::runtime_error("[PriorMap] YAML 'negate' must be 0 or 1 in '" +
                               yaml_path + "'");
    }
    prior.negate = negate == 1;

    if (root["ground_elevation"]) {
      const YAML::Node elevation = root["ground_elevation"];
      if (!elevation.IsMap()) {
        throw std::runtime_error(
            "[PriorMap] YAML 'ground_elevation' must be a map in '" +
            yaml_path + "'");
      }
      prior.ground_elevation_default_height =
          requireFiniteScalar(elevation, "default_height", yaml_path);
      if (elevation["grid"]) {
        const YAML::Node grid = elevation["grid"];
        if (!grid.IsMap()) {
          throw std::runtime_error(
              "[PriorMap] YAML 'ground_elevation.grid' must be a map in '" +
              yaml_path + "'");
        }
        if (!grid["image"] || !grid["image"].IsScalar()) {
          throw std::runtime_error(
              "[PriorMap] required YAML scalar 'ground_elevation.grid.image' "
              "is missing in '" +
              yaml_path + "'");
        }
        ground_elevation_grid_path = grid["image"].as<std::string>();
        if (ground_elevation_grid_path.is_relative()) {
          ground_elevation_grid_path =
              std::filesystem::path(yaml_path).parent_path() /
              ground_elevation_grid_path;
        }
        prior.ground_elevation_grid_scale =
            requireFiniteScalar(grid, "scale", yaml_path);
        prior.ground_elevation_grid_offset =
            requireFiniteScalar(grid, "offset", yaml_path);
        if (prior.ground_elevation_grid_scale <= 0.0) {
          throw std::runtime_error(
              "[PriorMap] YAML 'ground_elevation.grid.scale' must be positive "
              "in '" +
              yaml_path + "'");
        }
        if (!grid["no_data"] || !grid["no_data"].IsScalar()) {
          throw std::runtime_error(
              "[PriorMap] required YAML scalar 'ground_elevation.grid.no_data' "
              "is missing in '" +
              yaml_path + "'");
        }
        const int no_data = grid["no_data"].as<int>();
        if (no_data < 0 || no_data > 65535) {
          throw std::runtime_error(
              "[PriorMap] YAML 'ground_elevation.grid.no_data' must be in "
              "[0, 65535] in '" +
              yaml_path + "'");
        }
        prior.ground_elevation_grid_no_data =
            static_cast<uint16_t>(no_data);
        ground_elevation_grid_requested = true;
      }
      if (elevation["patches"] && !elevation["patches"].IsSequence()) {
        throw std::runtime_error(
            "[PriorMap] YAML 'ground_elevation.patches' must be a sequence in '" +
            yaml_path + "'");
      }
      if (elevation["patches"]) {
        for (const YAML::Node &node : elevation["patches"]) {
          if (!node.IsMap()) {
            throw std::runtime_error(
                "[PriorMap] every ground-elevation patch must be a map in '" +
                yaml_path + "'");
          }
          const auto bounds =
              requireFiniteSequence<4>(node, "bounds", yaml_path);
          const auto reference =
              requireFiniteSequence<3>(node, "reference", yaml_path);
          const auto slope =
              requireFiniteSequence<2>(node, "slope", yaml_path);
          if (bounds[0] >= bounds[2] || bounds[1] >= bounds[3]) {
            throw std::runtime_error(
                "[PriorMap] ground-elevation patch bounds must satisfy "
                "min_x < max_x and min_y < max_y in '" +
                yaml_path + "'");
          }
          prior.ground_elevation_patches.push_back(
              GroundElevationPatch{bounds[0], bounds[1], bounds[2], bounds[3],
                                   reference[0], reference[1], reference[2],
                                   slope[0], slope[1]});
        }
      }
      prior.ground_elevation_loaded = true;
    }
  } catch (const YAML::Exception &error) {
    throw std::runtime_error("[PriorMap] invalid YAML value in '" + yaml_path +
                             "': " + error.what());
  }

  if (prior.resolution <= 0.0) {
    throw std::runtime_error(
        "[PriorMap] YAML resolution must be positive in '" + yaml_path + "'");
  }
  if (prior.free_thresh < 0.0 || prior.free_thresh > 1.0 ||
      prior.occupied_thresh < 0.0 || prior.occupied_thresh > 1.0 ||
      prior.free_thresh >= prior.occupied_thresh) {
    throw std::runtime_error("[PriorMap] YAML thresholds must satisfy 0 <= "
                             "free_thresh < occupied_thresh <= 1 in '" +
                             yaml_path + "'");
  }

  std::filesystem::path image_path;
  if (!pgm_path.empty()) {
    image_path = pgm_path;
  } else {
    if (!root["image"] || !root["image"].IsScalar()) {
      throw std::runtime_error("[PriorMap] YAML 'image' is required when "
                               "projection.prior_map.pgm_path is empty");
    }
    image_path = root["image"].as<std::string>();
    if (image_path.is_relative()) {
      image_path = std::filesystem::path(yaml_path).parent_path() / image_path;
    }
  }

  const std::vector<uint8_t> pixels = loadPgm(
      image_path.lexically_normal().string(), prior.width, prior.height);
  prior.occupied.resize(pixels.size(), 0U);
  prior.known_free.resize(pixels.size(), 0U);
  for (size_t i = 0; i < pixels.size(); ++i) {
    const double gray = static_cast<double>(pixels[i]);
    const double occupancy =
        prior.negate ? gray / 255.0 : (255.0 - gray) / 255.0;
    prior.occupied[i] = occupancy > prior.occupied_thresh ? 1U : 0U;
    prior.known_free[i] = occupancy < prior.free_thresh ? 1U : 0U;
  }
  if (ground_elevation_grid_requested) {
    const std::string normalized_grid_path =
        ground_elevation_grid_path.lexically_normal().string();
    RawPgm grid = loadRawPgm(normalized_grid_path);
    if (grid.width != prior.width || grid.height != prior.height) {
      throw std::runtime_error(
          "[PriorMap] ground-elevation grid dimensions " +
          std::to_string(grid.width) + "x" + std::to_string(grid.height) +
          " do not match occupancy map dimensions " +
          std::to_string(prior.width) + "x" + std::to_string(prior.height) +
          " in '" + normalized_grid_path + "'");
    }
    if (prior.ground_elevation_grid_no_data > grid.max_value) {
      throw std::runtime_error(
          "[PriorMap] ground-elevation grid no_data exceeds its PGM maximum "
          "gray value in '" +
          normalized_grid_path + "'");
    }
    prior.ground_elevation_grid_width = grid.width;
    prior.ground_elevation_grid_height = grid.height;
    prior.ground_elevation_grid_max_value = grid.max_value;
    prior.ground_elevation_grid = std::move(grid.pixels);
    prior.ground_elevation_grid_loaded = true;
  }
  prior.loaded = true;
  return prior;
}

bool priorMapOccupied(const PriorMapData &prior_map, double map_x,
                      double map_y) {
  const double c = std::cos(prior_map.origin_yaw);
  const double s = std::sin(prior_map.origin_yaw);
  return priorMapStateWithRotation(prior_map, map_x, map_y, c, s) == 2U;
}

bool priorMapGroundSupportAtMapPoint(const PriorMapData &prior_map,
                                     double map_x, double map_y,
                                     double &support_z_map) {
  const double c = std::cos(prior_map.origin_yaw);
  const double s = std::sin(prior_map.origin_yaw);
  if (priorMapStateWithRotation(prior_map, map_x, map_y, c, s) != 1U) {
    return false;
  }
  return groundElevationAtMapPoint(prior_map, map_x, map_y, support_z_map);
}

bool priorMapGroundSupport(const PriorMapData &prior_map, double rog_x,
                           double rog_y, double &support_z_rog) {
  if (!prior_map.transform_ready ||
      std::abs(prior_map.fixed_transform.roll) > 1.0e-5 ||
      std::abs(prior_map.fixed_transform.pitch) > 1.0e-5) {
    return false;
  }
  double map_x = 0.0;
  double map_y = 0.0;
  transformPriorMapPoint(prior_map.fixed_transform, rog_x, rog_y, map_x,
                         map_y);
  if (priorMapStateWithRotation(prior_map, map_x, map_y,
                                prior_map.fast_origin_yaw_cos,
                                prior_map.fast_origin_yaw_sin) != 1U) {
    return false;
  }
  double support_z_map = 0.0;
  if (!groundElevationAtMapPoint(prior_map, map_x, map_y, support_z_map)) {
    return false;
  }
  support_z_rog = support_z_map - prior_map.fixed_transform.tz;
  return std::isfinite(support_z_rog);
}

void transformPriorMapPoint(const PriorMapTransform2D &transform, double rog_x,
                            double rog_y, double &map_x, double &map_y) {
  const double c = std::cos(transform.yaw);
  const double s = std::sin(transform.yaw);
  map_x = c * rog_x - s * rog_y + transform.tx;
  map_y = s * rog_x + c * rog_y + transform.ty;
}

bool updatePriorMapTransform(PriorMapData &prior_map,
                             const PriorMapTransform2D &transform) {
  if (!prior_map.loaded) {
    return false;
  }
  if (!std::isfinite(transform.tx) || !std::isfinite(transform.ty) ||
      !std::isfinite(transform.yaw) || !std::isfinite(transform.tz) ||
      !std::isfinite(transform.roll) || !std::isfinite(transform.pitch) ||
      !std::isfinite(prior_map.origin_yaw) ||
      std::abs(transform.roll) > 1.0e-5 ||
      std::abs(transform.pitch) > 1.0e-5) {
    return invalidatePriorMapTransform(prior_map);
  }

  if (prior_map.transform_ready) {
    constexpr double kTwoPi = 6.28318530717958647692;
    const double yaw_delta =
        std::remainder(transform.yaw - prior_map.fixed_transform.yaw, kTwoPi);
    constexpr double kTransformEpsilon = 1.0e-9;
    if (std::abs(transform.tx - prior_map.fixed_transform.tx) <=
            kTransformEpsilon &&
        std::abs(transform.ty - prior_map.fixed_transform.ty) <=
            kTransformEpsilon &&
        std::abs(yaw_delta) <= kTransformEpsilon &&
        std::abs(transform.tz - prior_map.fixed_transform.tz) <=
            kTransformEpsilon) {
      return false;
    }
  }

  prior_map.fixed_transform = transform;
  prior_map.fixed_transform_cos = std::cos(transform.yaw);
  prior_map.fixed_transform_sin = std::sin(transform.yaw);
  prior_map.fast_origin_yaw_cos = std::cos(prior_map.origin_yaw);
  prior_map.fast_origin_yaw_sin = std::sin(prior_map.origin_yaw);
  prior_map.transform_ready = true;
  prior_map.projection_cache_ready = false;
  return true;
}

bool invalidatePriorMapTransform(PriorMapData &prior_map) {
  const bool changed =
      prior_map.transform_ready || prior_map.projection_cache_ready;
  prior_map.transform_ready = false;
  prior_map.projection_cache_ready = false;
  return changed;
}

bool refreshPriorMapProjectionCache(PriorMapData &prior_map, int min_global_x,
                                    int min_global_y, int width, int height,
                                    double resolution, double origin_x,
                                    double origin_y) {
  const size_t expected_size = checkedCellCount(width, height);
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(origin_x) || !std::isfinite(origin_y)) {
    throw std::invalid_argument(
        "PriorMap projection geometry must be finite and positive");
  }

  const bool same_geometry =
      sameProjectionGeometry(prior_map, min_global_x, min_global_y, width,
                             height, resolution, expected_size);
  if (!prior_map.transform_ready) {
    if (same_geometry && !prior_map.projection_cache_ready) {
      return false;
    }
    prior_map.cached_mask.assign(expected_size, 1U);
    prior_map.cached_free_mask.assign(expected_size, 0U);
    prior_map.cached_ground_support_mask.assign(expected_size, 0U);
    prior_map.cached_ground_support_z.assign(
        expected_size, std::numeric_limits<float>::quiet_NaN());
    prior_map.scratch_mask.resize(expected_size);
    prior_map.scratch_free_mask.resize(expected_size);
    prior_map.scratch_ground_support_mask.resize(expected_size);
    prior_map.scratch_ground_support_z.resize(expected_size);
    saveProjectionGeometry(prior_map, min_global_x, min_global_y, width, height,
                           resolution);
    prior_map.projection_cache_ready = false;
    return true;
  }

  if (prior_map.projection_cache_ready && same_geometry) {
    return false;
  }

  const bool reusable_geometry =
      prior_map.projection_cache_ready && prior_map.cached_width == width &&
      prior_map.cached_height == height &&
      std::abs(prior_map.cached_resolution - resolution) <= 1.0e-9 &&
      prior_map.cached_mask.size() == expected_size &&
      prior_map.cached_free_mask.size() == expected_size &&
      prior_map.cached_ground_support_mask.size() == expected_size &&
      prior_map.cached_ground_support_z.size() == expected_size;
  if (!reusable_geometry) {
    rebuildPriorProjection(prior_map, min_global_x, min_global_y, width, height,
                           resolution, origin_x, origin_y, expected_size);
    return true;
  }

  const int64_t old_min_x = prior_map.cached_min_x;
  const int64_t old_min_y = prior_map.cached_min_y;
  const int64_t new_min_x = min_global_x;
  const int64_t new_min_y = min_global_y;
  const int64_t old_max_x = old_min_x + static_cast<int64_t>(width) - 1;
  const int64_t old_max_y = old_min_y + static_cast<int64_t>(height) - 1;
  const int64_t new_max_x = new_min_x + static_cast<int64_t>(width) - 1;
  const int64_t new_max_y = new_min_y + static_cast<int64_t>(height) - 1;
  const int64_t overlap_min_x = std::max(old_min_x, new_min_x);
  const int64_t overlap_min_y = std::max(old_min_y, new_min_y);
  const int64_t overlap_max_x = std::min(old_max_x, new_max_x);
  const int64_t overlap_max_y = std::min(old_max_y, new_max_y);
  const bool has_overlap =
      overlap_min_x <= overlap_max_x && overlap_min_y <= overlap_max_y;
  if (!has_overlap) {
    rebuildPriorProjection(prior_map, min_global_x, min_global_y, width, height,
                           resolution, origin_x, origin_y, expected_size);
    return true;
  }

  prior_map.scratch_mask.resize(expected_size);
  std::fill(prior_map.scratch_mask.begin(), prior_map.scratch_mask.end(), 1U);
  prior_map.scratch_free_mask.resize(expected_size);
  std::fill(prior_map.scratch_free_mask.begin(),
            prior_map.scratch_free_mask.end(), 0U);
  prior_map.scratch_ground_support_mask.resize(expected_size);
  std::fill(prior_map.scratch_ground_support_mask.begin(),
            prior_map.scratch_ground_support_mask.end(), 0U);
  prior_map.scratch_ground_support_z.resize(expected_size);
  std::fill(prior_map.scratch_ground_support_z.begin(),
            prior_map.scratch_ground_support_z.end(),
            std::numeric_limits<float>::quiet_NaN());
  const size_t overlap_width =
      static_cast<size_t>(overlap_max_x - overlap_min_x + 1);
  for (int64_t global_y = overlap_min_y; global_y <= overlap_max_y;
       ++global_y) {
    const size_t old_y = static_cast<size_t>(global_y - old_min_y);
    const size_t new_y = static_cast<size_t>(global_y - new_min_y);
    const size_t old_x = static_cast<size_t>(overlap_min_x - old_min_x);
    const size_t new_x = static_cast<size_t>(overlap_min_x - new_min_x);
    std::copy_n(prior_map.cached_mask.begin() +
                    old_y * static_cast<size_t>(width) + old_x,
                overlap_width,
                prior_map.scratch_mask.begin() +
                    new_y * static_cast<size_t>(width) + new_x);
    std::copy_n(prior_map.cached_free_mask.begin() +
                    old_y * static_cast<size_t>(width) + old_x,
                overlap_width,
                prior_map.scratch_free_mask.begin() +
                    new_y * static_cast<size_t>(width) + new_x);
    std::copy_n(prior_map.cached_ground_support_mask.begin() +
                    old_y * static_cast<size_t>(width) + old_x,
                overlap_width,
                prior_map.scratch_ground_support_mask.begin() +
                    new_y * static_cast<size_t>(width) + new_x);
    std::copy_n(prior_map.cached_ground_support_z.begin() +
                    old_y * static_cast<size_t>(width) + old_x,
                overlap_width,
                prior_map.scratch_ground_support_z.begin() +
                    new_y * static_cast<size_t>(width) + new_x);
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int64_t global_x = new_min_x + x;
      const int64_t global_y = new_min_y + y;
      if (global_x >= overlap_min_x && global_x <= overlap_max_x &&
          global_y >= overlap_min_y && global_y <= overlap_max_y) {
        continue;
      }
      const double rog_x =
          origin_x + (static_cast<double>(x) + 0.5) * resolution;
      const double rog_y =
          origin_y + (static_cast<double>(y) + 0.5) * resolution;
      const size_t index = static_cast<size_t>(y) * static_cast<size_t>(width) +
                           static_cast<size_t>(x);
      const uint8_t prior_state =
          samplePriorCellConservative(prior_map, rog_x, rog_y, resolution);
      if (prior_state == 2U) {
        prior_map.scratch_mask[index] = 0U;
      } else if (prior_state == 1U) {
        prior_map.scratch_free_mask[index] = 1U;
        double support_z_rog = 0.0;
        if (priorMapGroundSupport(prior_map, rog_x, rog_y, support_z_rog)) {
          prior_map.scratch_ground_support_mask[index] = 1U;
          prior_map.scratch_ground_support_z[index] =
              static_cast<float>(support_z_rog);
        }
      }
    }
  }

  prior_map.cached_mask.swap(prior_map.scratch_mask);
  prior_map.cached_free_mask.swap(prior_map.scratch_free_mask);
  prior_map.cached_ground_support_mask.swap(
      prior_map.scratch_ground_support_mask);
  prior_map.cached_ground_support_z.swap(prior_map.scratch_ground_support_z);
  saveProjectionGeometry(prior_map, min_global_x, min_global_y, width, height,
                         resolution);
  return true;
}

void fusePriorMapProjection(
    bool prior_enabled, bool free_fills_unknown, const PriorMapData &prior_map,
    const std::vector<uint8_t> &dynamic_mask,
    const std::vector<uint8_t> &dynamic_values,
    const std::vector<uint8_t> &dynamic_unknown_mask,
    const std::vector<uint8_t> &dynamic_near_field_prior_fill_mask,
    std::vector<uint8_t> &fused_mask, std::vector<uint8_t> &fused_values,
    bool require_ground_support) {
  if (dynamic_mask.size() != dynamic_values.size() ||
      dynamic_mask.size() != dynamic_unknown_mask.size() ||
      dynamic_mask.size() != dynamic_near_field_prior_fill_mask.size()) {
    fused_mask.clear();
    fused_values.clear();
    throw std::invalid_argument(
        "fusePriorMapProjection: dynamic mask, value, unknown-mask, and "
        "near-field-mask sizes differ");
  }

  fused_mask = dynamic_mask;
  fused_values = dynamic_values;
  if (!prior_enabled) {
    return;
  }
  if (!prior_map.loaded || !prior_map.transform_ready ||
      !prior_map.projection_cache_ready ||
      prior_map.cached_mask.size() != dynamic_mask.size() ||
      prior_map.cached_free_mask.size() != dynamic_mask.size()) {
    std::fill(fused_mask.begin(), fused_mask.end(), 0U);
    std::fill(fused_values.begin(), fused_values.end(), 254U);
    return;
  }

  for (size_t index = 0; index < prior_map.cached_mask.size(); ++index) {
    if (prior_map.cached_mask[index] == 0U) {
      fused_mask[index] = 0U;
      fused_values[index] = 254U;
    } else if (!require_ground_support &&
               prior_map.cached_free_mask[index] != 0U &&
               ((free_fills_unknown && dynamic_unknown_mask[index] != 0U) ||
                dynamic_near_field_prior_fill_mask[index] != 0U)) {
      // Global unknown filling is optional. The bounded near-field path is
      // independent and has already rejected every column with a dynamic hit.
      fused_mask[index] = 1U;
      fused_values[index] = 0U;
    }
  }
}

} // namespace rog_map
