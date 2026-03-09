#include "simulation_core/track_loader.hpp"

#include <cmath>
#include <fstream>

#include <yaml-cpp/yaml.h>

namespace simulation_core {

bool TrackLoader::load(const std::string& filepath, Track& track) {
  try {
    YAML::Node config = YAML::LoadFile(filepath);

    if (!config["track"]) {
      last_error_ = "Missing 'track' key in YAML file";
      return false;
    }

    auto track_node = config["track"];

    // Load track name
    track.name = track_node["name"].as<std::string>("unnamed");

    // Load start position
    if (track_node["start"]) {
      track.start_x = track_node["start"]["x"].as<double>(0.0);
      track.start_y = track_node["start"]["y"].as<double>(0.0);
      track.start_heading = track_node["start"]["heading"].as<double>(0.0);
    }

    // Load cones
    track.cones.clear();

    if (track_node["cones"]) {
      auto cones_node = track_node["cones"];

      // Blue cones (right boundary)
      if (cones_node["blue"]) {
        for (const auto& pos : cones_node["blue"]) {
          Cone cone;
          cone.x = pos[0].as<double>();
          cone.y = pos[1].as<double>();
          cone.color = ConeColor::BLUE;
          track.cones.push_back(cone);
        }
      }

      // Yellow small cones (left boundary - was 'yellow')
      if (cones_node["yellow"]) {
        for (const auto& pos : cones_node["yellow"]) {
          Cone cone;
          cone.x = pos[0].as<double>();
          cone.y = pos[1].as<double>();
          cone.color = ConeColor::YELLOW_SMALL;
          track.cones.push_back(cone);
        }
      }

      // Yellow small cones (was orange - start/finish markers)
      if (cones_node["yellow_small"]) {
        for (const auto& pos : cones_node["yellow_small"]) {
          Cone cone;
          cone.x = pos[0].as<double>();
          cone.y = pos[1].as<double>();
          cone.color = ConeColor::YELLOW_SMALL;
          track.cones.push_back(cone);
        }
      }

      // Yellow big cones (was orange_big - large start/finish markers)
      if (cones_node["yellow_big"]) {
        for (const auto& pos : cones_node["yellow_big"]) {
          Cone cone;
          cone.x = pos[0].as<double>();
          cone.y = pos[1].as<double>();
          cone.color = ConeColor::YELLOW_BIG;
          track.cones.push_back(cone);
        }
      }
    }

    return true;
  } catch (const YAML::Exception& e) {
    last_error_ = std::string("YAML parsing error: ") + e.what();
    return false;
  } catch (const std::exception& e) {
    last_error_ = std::string("Error loading track: ") + e.what();
    return false;
  }
}

bool TrackLoader::save(const std::string& filepath, const Track& track) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "track" << YAML::Value << YAML::BeginMap;

    // Name
    out << YAML::Key << "name" << YAML::Value << track.name;

    // Start position
    out << YAML::Key << "start" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << track.start_x;
    out << YAML::Key << "y" << YAML::Value << track.start_y;
    out << YAML::Key << "heading" << YAML::Value << track.start_heading;
    out << YAML::EndMap;

    // Cones by color
    out << YAML::Key << "cones" << YAML::Value << YAML::BeginMap;

    // Collect cones by color
    std::vector<Cone> blue, yellow, yellow_small, yellow_big;
    for (const auto& cone : track.cones) {
      switch (cone.color) {
        case ConeColor::BLUE:
          blue.push_back(cone);
          break;
        case ConeColor::YELLOW_SMALL:
          yellow_small.push_back(cone);
          break;
        case ConeColor::YELLOW_BIG:
          yellow_big.push_back(cone);
          break;
        default:
          break;
      }
    }

    auto writeCones = [&out](const std::string& name, const std::vector<Cone>& cones) {
      if (!cones.empty()) {
        out << YAML::Key << name << YAML::Value << YAML::BeginSeq;
        for (const auto& c : cones) {
          out << YAML::Flow << YAML::BeginSeq << c.x << c.y << YAML::EndSeq;
        }
        out << YAML::EndSeq;
      }
    };

    writeCones("blue", blue);
    writeCones("yellow_small", yellow_small);
    writeCones("yellow_big", yellow_big);

    out << YAML::EndMap;  // cones
    out << YAML::EndMap;  // track
    out << YAML::EndMap;  // root

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
      last_error_ = "Failed to open file for writing: " + filepath;
      return false;
    }
    fout << out.c_str();
    return true;
  } catch (const std::exception& e) {
    last_error_ = std::string("Error saving track: ") + e.what();
    return false;
  }
}

Track TrackLoader::createOvalTrack(double length, double width, double cone_spacing) {
  Track track;
  track.name = "oval";
  track.start_x = 0.0;
  track.start_y = 0.0;
  track.start_heading = 0.0;

  double half_width = width / 2.0;
  double straight_length = length / 2.0;
  double turn_radius = half_width;

  // Straight sections
  for (double x = 0; x <= straight_length; x += cone_spacing) {
    // Right side (blue)
    Cone blue;
    blue.x = x;
    blue.y = -half_width;
    blue.color = ConeColor::BLUE;
    track.cones.push_back(blue);

    // Left side (yellow_small)
    Cone yellow_small;
    yellow_small.x = x;
    yellow_small.y = half_width;
    yellow_small.color = ConeColor::YELLOW_SMALL;
    track.cones.push_back(yellow_small);

    // Back straight
    blue.x = -x;
    track.cones.push_back(blue);
    yellow_small.x = -x;
    track.cones.push_back(yellow_small);
  }

  // Turn sections (semicircles at each end)
  int num_turn_cones = static_cast<int>(M_PI * turn_radius / cone_spacing);
  for (int i = 1; i < num_turn_cones; ++i) {
    double angle = M_PI * i / num_turn_cones;

    // Front turn (right side)
    double cx = straight_length;
    Cone blue, yellow_small;

    blue.x = cx + turn_radius * std::sin(angle);
    blue.y = -turn_radius * std::cos(angle);
    blue.color = ConeColor::BLUE;
    track.cones.push_back(blue);

    yellow_small.x = cx + (turn_radius + width) * std::sin(angle);
    yellow_small.y = -(turn_radius + width) * std::cos(angle) + width;
    yellow_small.color = ConeColor::YELLOW_SMALL;
    track.cones.push_back(yellow_small);

    // Back turn
    blue.x = -cx - turn_radius * std::sin(angle);
    track.cones.push_back(blue);
    yellow_small.x = -cx - (turn_radius + width) * std::sin(angle);
    track.cones.push_back(yellow_small);
  }

  // Start/finish cones (yellow_small)
  Cone sf1, sf2;
  sf1.x = 0.0;
  sf1.y = -half_width - 0.5;
  sf1.color = ConeColor::YELLOW_SMALL;
  sf2.x = 0.0;
  sf2.y = half_width + 0.5;
  sf2.color = ConeColor::YELLOW_SMALL;
  track.cones.push_back(sf1);
  track.cones.push_back(sf2);

  return track;
}

Track TrackLoader::createSkidpadTrack(double radius, double cone_spacing) {
  Track track;
  track.name = "skidpad";
  track.start_x = 0.0;
  track.start_y = 0.0;
  track.start_heading = 0.0;

  double track_width = 3.0;  // Standard skidpad track width
  double inner_radius = radius - track_width / 2.0;
  double outer_radius = radius + track_width / 2.0;

  // Right circle (centered at (radius, 0))
  int num_cones = static_cast<int>(2 * M_PI * radius / cone_spacing);
  for (int i = 0; i < num_cones; ++i) {
    double angle = 2 * M_PI * i / num_cones;

    // Inner circle (blue - right boundary)
    Cone blue;
    blue.x = radius + inner_radius * std::cos(angle);
    blue.y = inner_radius * std::sin(angle);
    blue.color = ConeColor::BLUE;
    track.cones.push_back(blue);

    // Outer circle (yellow_small - left boundary)
    Cone yellow_small;
    yellow_small.x = radius + outer_radius * std::cos(angle);
    yellow_small.y = outer_radius * std::sin(angle);
    yellow_small.color = ConeColor::YELLOW_SMALL;
    track.cones.push_back(yellow_small);
  }

  // Left circle (centered at (-radius, 0))
  for (int i = 0; i < num_cones; ++i) {
    double angle = 2 * M_PI * i / num_cones;

    // Inner circle (yellow_small - left boundary for this circle)
    Cone yellow_small;
    yellow_small.x = -radius + inner_radius * std::cos(angle);
    yellow_small.y = inner_radius * std::sin(angle);
    yellow_small.color = ConeColor::YELLOW_SMALL;
    track.cones.push_back(yellow_small);

    // Outer circle (blue - right boundary for this circle)
    Cone blue;
    blue.x = -radius + outer_radius * std::cos(angle);
    blue.y = outer_radius * std::sin(angle);
    blue.color = ConeColor::BLUE;
    track.cones.push_back(blue);
  }

  // Entry/exit cones (yellow_small)
  Cone entry1, entry2;
  entry1.x = 0.0;
  entry1.y = -track_width / 2.0 - 0.5;
  entry1.color = ConeColor::YELLOW_SMALL;
  entry2.x = 0.0;
  entry2.y = track_width / 2.0 + 0.5;
  entry2.color = ConeColor::YELLOW_SMALL;
  track.cones.push_back(entry1);
  track.cones.push_back(entry2);

  return track;
}

Track TrackLoader::createAccelerationTrack(double length, double width, double cone_spacing) {
  Track track;
  track.name = "acceleration";
  track.start_x = 0.0;
  track.start_y = 0.0;
  track.start_heading = 0.0;

  double half_width = width / 2.0;

  // Straight track with cones on both sides
  for (double x = 0; x <= length; x += cone_spacing) {
    // Right side (blue)
    Cone blue;
    blue.x = x;
    blue.y = -half_width;
    blue.color = ConeColor::BLUE;
    track.cones.push_back(blue);

    // Left side (yellow_small)
    Cone yellow_small;
    yellow_small.x = x;
    yellow_small.y = half_width;
    yellow_small.color = ConeColor::YELLOW_SMALL;
    track.cones.push_back(yellow_small);
  }

  // Start cones (yellow_small)
  Cone start1, start2;
  start1.x = 0.0;
  start1.y = -half_width - 0.5;
  start1.color = ConeColor::YELLOW_SMALL;
  start2.x = 0.0;
  start2.y = half_width + 0.5;
  start2.color = ConeColor::YELLOW_SMALL;
  track.cones.push_back(start1);
  track.cones.push_back(start2);

  // Finish cones (yellow_big)
  Cone finish1, finish2;
  finish1.x = length;
  finish1.y = -half_width - 0.5;
  finish1.color = ConeColor::YELLOW_BIG;
  finish2.x = length;
  finish2.y = half_width + 0.5;
  finish2.color = ConeColor::YELLOW_BIG;
  track.cones.push_back(finish1);
  track.cones.push_back(finish2);

  return track;
}

}  // namespace simulation_core
