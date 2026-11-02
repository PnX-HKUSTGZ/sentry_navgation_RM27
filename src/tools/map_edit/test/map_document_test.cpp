#include <gtest/gtest.h>
#include "map_edit/map_document.h"
#include <nav2_map_server/map_io.hpp>
#include <QImage>
#include <QTemporaryDir>
#include <QFile>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
namespace {
void textFile(const fs::path &path, const std::string &text) { std::ofstream(path) << text; }
std::string readFile(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
void makeFixture(const fs::path &path, const std::string &mode, int negate) {
  QImage pixels(256, 2, QImage::Format_ARGB32);
  for (int x=0; x<256; ++x) {
    pixels.setPixel(x, 0, qRgba(x,x,x,255));
    pixels.setPixel(x, 1, qRgba(x,x,x,x == 127 ? 0 : 255));
  }
  ASSERT_TRUE(pixels.save(QString::fromStdString((path / "source.png").string()), "PNG"));
  textFile(path / "map.yaml", "image: source.png\nmode: " + mode +
    "\nresolution: 0.05\norigin: [-4.2, 1.8, 0.73]\nnegate: " + std::to_string(negate) +
    "\noccupied_thresh: 0.71\nfree_thresh: 0.29\nground_elevation:\n"
    "  default_height: 0.0\n  grid:\n    image: source_elevation.pgm\n"
    "    scale: 0.001\n    offset: -1.0\n    no_data: 65535\n  patches: []\n"
    "extension:\n  nested: [a, 42, true]\n  owner: preserve-me\n");
  textFile(path / "source_elevation.pgm", "untouched elevation asset");
}
}

TEST(MapDocument, MetadataOccupancyNegateModesAndSourceProtection) {
  for (const auto mode : {"trinary", "raw", "scale"}) {
    for (int negate : {0,1}) {
      SCOPED_TRACE(std::string(mode) + "/" + std::to_string(negate));
      QTemporaryDir directory;
      const fs::path path(directory.path().toStdString());
      makeFixture(path, mode, negate);
      const auto before_yaml = readFile(path / "map.yaml");
      const auto before_image = readFile(path / "source.png");
      const auto before_elevation = readFile(path / "source_elevation.pgm");
      map_edit::MapDocument doc;
      auto map = doc.load((path / "map.yaml").string());
      // Exercise editor-compatible changed cells plus all decoded intermediate values.
      map.data[0] = 100; map.data[1] = 0; map.data[2] = -1;
      fs::create_directories(path / "export" / "nested");
      const auto output = path / "export" / "nested" / "edited.yaml";
      const auto image = doc.save(output.string(), map);
      EXPECT_EQ(fs::path(image).extension(), std::string(mode)=="scale" ? ".png" : ".pgm");
      map_edit::MapDocument round_trip;
      const auto reloaded = round_trip.load(output.string());
      EXPECT_EQ(map.data, reloaded.data);
      EXPECT_EQ(map.info.width, reloaded.info.width);
      EXPECT_EQ(map.info.height, reloaded.info.height);
      EXPECT_FLOAT_EQ(map.info.resolution, reloaded.info.resolution);
      EXPECT_DOUBLE_EQ(map.info.origin.orientation.z, reloaded.info.origin.orientation.z);
      const auto metadata = YAML::LoadFile(output.string());
      EXPECT_EQ(metadata["mode"].as<std::string>(), mode);
      EXPECT_EQ(metadata["negate"].as<int>(), negate);
      EXPECT_DOUBLE_EQ(metadata["occupied_thresh"].as<double>(), .71);
      EXPECT_DOUBLE_EQ(metadata["free_thresh"].as<double>(), .29);
      EXPECT_EQ(YAML::Dump(metadata["extension"]), YAML::Dump(doc.metadata()["extension"]));
      EXPECT_EQ(fs::weakly_canonical(output.parent_path() /
        metadata["ground_elevation"]["grid"]["image"].as<std::string>()),
        fs::weakly_canonical(path / "source_elevation.pgm"));
      EXPECT_DOUBLE_EQ(metadata["ground_elevation"]["grid"]["scale"].as<double>(), .001);
      EXPECT_THROW(doc.save((path / "map.yaml").string(), map), std::runtime_error);
      EXPECT_THROW(doc.save(output.string(), map), std::runtime_error);
      EXPECT_THROW(doc.save((path / "source_elevation.pgm").string(), map), std::runtime_error);
      EXPECT_EQ(before_yaml, readFile(path / "map.yaml"));
      EXPECT_EQ(before_image, readFile(path / "source.png"));
      EXPECT_EQ(before_elevation, readFile(path / "source_elevation.pgm"));
    }
  }
}

TEST(MapDocument, RepositoryRMUC2026ElevationCrossDirectorySave) {
  const fs::path source(MAP_EDIT_REPOSITORY_FIXTURE);
  ASSERT_TRUE(fs::exists(source));
  const auto metadata = YAML::LoadFile(source.string());
  const auto image = source.parent_path() / metadata["image"].as<std::string>();
  const auto elevation = source.parent_path() /
    metadata["ground_elevation"]["grid"]["image"].as<std::string>();
  const auto before_yaml = readFile(source), before_image = readFile(image), before_elevation = readFile(elevation);
  map_edit::MapDocument doc;
  const auto original = doc.load(source.string());
  QTemporaryDir directory;
  const fs::path output = fs::path(directory.path().toStdString()) / "edited.yaml";
  doc.save(output.string(), original);
  map_edit::MapDocument reload;
  EXPECT_EQ(reload.load(output.string()).data, original.data);
  const auto exported = YAML::LoadFile(output.string());
  EXPECT_EQ(fs::weakly_canonical(output.parent_path() /
    exported["ground_elevation"]["grid"]["image"].as<std::string>()), fs::weakly_canonical(elevation));
  EXPECT_EQ(YAML::Dump(exported["ground_elevation"]["patches"]), YAML::Dump(metadata["ground_elevation"]["patches"]));
  EXPECT_EQ(before_yaml, readFile(source)); EXPECT_EQ(before_image, readFile(image));
  EXPECT_EQ(before_elevation, readFile(elevation));
}
