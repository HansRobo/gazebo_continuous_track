#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <sdf/Element.hh>
#include <sdf/Joint.hh>

namespace gazebo_continuous_track {

struct SegmentConfig {
  gz::sim::Entity joint_entity{gz::sim::kNullEntity};
  double velocity_scale{1.0};
  bool reset_position{false};
};

class ContinuousTrackSystem : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr< const sdf::Element > &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override {
    this->configured_ = false;
    this->segments_.clear();
    this->sprocket_joint_entity_ = gz::sim::kNullEntity;

    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      gzerr << "[ContinuousTrackSystem] Target entity is not a model." << std::endl;
      return;
    }
    if (!_sdf) {
      gzerr << "[ContinuousTrackSystem] Missing plugin SDF." << std::endl;
      return;
    }

    auto plugin_sdf = _sdf->Clone();
    if (!plugin_sdf->HasElement("sprocket")) {
      gzerr << "[ContinuousTrackSystem] Missing required <sprocket> element." << std::endl;
      return;
    }

    auto sprocket_elem = plugin_sdf->GetElement("sprocket");
    if (!sprocket_elem->HasElement("joint") || !sprocket_elem->HasElement("pitch_diameter")) {
      gzerr << "[ContinuousTrackSystem] <sprocket> must contain <joint> and "
            << "<pitch_diameter>." << std::endl;
      return;
    }

    const auto sprocket_joint_name = sprocket_elem->Get< std::string >("joint");
    const double sprocket_pitch_diameter = sprocket_elem->Get< double >("pitch_diameter");
    if (sprocket_pitch_diameter <= 0.0) {
      gzerr << "[ContinuousTrackSystem] <sprocket><pitch_diameter> must be positive."
            << std::endl;
      return;
    }

    this->sprocket_joint_entity_ = this->model_.JointByName(_ecm, sprocket_joint_name);
    if (this->sprocket_joint_entity_ == gz::sim::kNullEntity) {
      gzerr << "[ContinuousTrackSystem] Joint not found: " << sprocket_joint_name << std::endl;
      return;
    }
    gz::sim::Joint sprocket_joint(this->sprocket_joint_entity_);
    sprocket_joint.EnableVelocityCheck(_ecm, true);

    if (plugin_sdf->HasElement("track")) {
      this->LoadTrackSegments(plugin_sdf->GetElement("track"), sprocket_pitch_diameter, _ecm);
    } else if (plugin_sdf->HasElement("trajectory")) {
      this->LoadTrajectorySegments(plugin_sdf->GetElement("trajectory"), sprocket_pitch_diameter,
                                   _ecm);
    } else {
      gzerr << "[ContinuousTrackSystem] Missing <track> or <trajectory> element." << std::endl;
      return;
    }

    if (this->segments_.empty()) {
      gzerr << "[ContinuousTrackSystem] No valid segments found." << std::endl;
      return;
    }

    if (plugin_sdf->HasElement("pattern")) {
      gzwarn << "[ContinuousTrackSystem] <pattern> is accepted but geometry pattern switching is "
             << "not implemented in this ROS 2/Harmonic port." << std::endl;
    }

    this->configured_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override {
    if (!this->configured_ || _info.paused) {
      return;
    }

    gz::sim::Joint sprocket_joint(this->sprocket_joint_entity_);
    const auto sprocket_velocity = sprocket_joint.Velocity(_ecm);
    if (!sprocket_velocity || sprocket_velocity->empty()) {
      return;
    }
    const double sprocket_vel = sprocket_velocity->at(0);

    for (const auto &segment : this->segments_) {
      gz::sim::Joint segment_joint(segment.joint_entity);
      if (segment.reset_position) {
        segment_joint.ResetPosition(_ecm, {0.0});
      }
      segment_joint.SetVelocity(_ecm, {sprocket_vel * segment.velocity_scale});
    }
  }

private:
  void LoadTrackSegments(const sdf::ElementPtr &_track_elem, const double _sprocket_pitch_diameter,
                         gz::sim::EntityComponentManager &_ecm) {
    if (!_track_elem->HasElement("segment")) {
      gzerr << "[ContinuousTrackSystem] <track> requires at least one <segment>." << std::endl;
      return;
    }

    for (auto segment_elem = _track_elem->GetElement("segment"); segment_elem;
         segment_elem = segment_elem->GetNextElement("segment")) {
      this->LoadOneSegment(segment_elem, _sprocket_pitch_diameter, _ecm);
    }
  }

  void LoadTrajectorySegments(const sdf::ElementPtr &_traj_elem,
                              const double _sprocket_pitch_diameter,
                              gz::sim::EntityComponentManager &_ecm) {
    if (!_traj_elem->HasElement("segment")) {
      gzerr << "[ContinuousTrackSystem] <trajectory> requires at least one <segment>."
            << std::endl;
      return;
    }

    for (auto segment_elem = _traj_elem->GetElement("segment"); segment_elem;
         segment_elem = segment_elem->GetNextElement("segment")) {
      this->LoadOneSegment(segment_elem, _sprocket_pitch_diameter, _ecm);
    }
  }

  void LoadOneSegment(const sdf::ElementPtr &_segment_elem, const double _sprocket_pitch_diameter,
                      gz::sim::EntityComponentManager &_ecm) {
    if (!_segment_elem->HasElement("joint")) {
      gzerr << "[ContinuousTrackSystem] <segment> is missing required <joint>." << std::endl;
      return;
    }

    const auto segment_joint_name = _segment_elem->Get< std::string >("joint");
    const auto segment_joint_entity = this->model_.JointByName(_ecm, segment_joint_name);
    if (segment_joint_entity == gz::sim::kNullEntity) {
      gzerr << "[ContinuousTrackSystem] Joint not found: " << segment_joint_name << std::endl;
      return;
    }

    gz::sim::Joint segment_joint(segment_joint_entity);
    const auto segment_type = segment_joint.Type(_ecm);
    if (!segment_type) {
      gzerr << "[ContinuousTrackSystem] Could not resolve joint type for: " << segment_joint_name
            << std::endl;
      return;
    }

    SegmentConfig segment;
    segment.joint_entity = segment_joint_entity;

    if (*segment_type == sdf::JointType::REVOLUTE || *segment_type == sdf::JointType::CONTINUOUS) {
      // For the original ContinuousTrack plugin, this scale is derived from
      // geometry. The ROS 2/Harmonic port falls back to optional
      // <pitch_diameter> on each segment or 1.0 ratio if omitted.
      double segment_pitch_diameter = _sprocket_pitch_diameter;
      if (_segment_elem->HasElement("pitch_diameter")) {
        segment_pitch_diameter = _segment_elem->Get< double >("pitch_diameter");
      }
      if (segment_pitch_diameter <= 0.0) {
        gzerr << "[ContinuousTrackSystem] Segment pitch diameter must be positive for: "
              << segment_joint_name << std::endl;
        return;
      }
      segment.velocity_scale = _sprocket_pitch_diameter / segment_pitch_diameter;
    } else if (*segment_type == sdf::JointType::PRISMATIC) {
      segment.velocity_scale = _sprocket_pitch_diameter / 2.0;
      segment.reset_position = true;
      segment_joint.EnablePositionCheck(_ecm, true);
    } else {
      gzerr << "[ContinuousTrackSystem] Joint type must be revolute/continuous/prismatic: "
            << segment_joint_name << std::endl;
      return;
    }

    this->segments_.push_back(segment);
  }

private:
  bool configured_{false};
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity sprocket_joint_entity_{gz::sim::kNullEntity};
  std::vector< SegmentConfig > segments_;
};

} // namespace gazebo_continuous_track

GZ_ADD_PLUGIN(gazebo_continuous_track::ContinuousTrackSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gazebo_continuous_track::ContinuousTrackSystem,
                    "gazebo_continuous_track::ContinuousTrackSystem")
