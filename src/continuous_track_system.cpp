#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Transparency.hh>

#include <sdf/Collision.hh>
#include <sdf/Element.hh>
#include <sdf/Joint.hh>
#include <sdf/Surface.hh>
#include <sdf/Visual.hh>

namespace gazebo_continuous_track {

struct PatternElementConfig {
  std::vector< sdf::Visual > visuals;
  std::vector< sdf::Collision > collisions;
};

struct PatternConfig {
  bool enabled{false};
  std::size_t elements_per_round{0};
  std::vector< PatternElementConfig > elements;
  std::size_t active_variant{0};
  double perimeter{0.0};
  double len_per_element{0.0};
};

struct SegmentConfig {
  gz::sim::Entity joint_entity{gz::sim::kNullEntity};
  gz::sim::Entity child_link_entity{gz::sim::kNullEntity};
  sdf::JointType type{sdf::JointType::INVALID};

  // joint velocity command scale from sprocket rotational speed [rad/s]
  // to this segment joint speed.
  double velocity_scale{1.0};
  bool reset_position{false};

  // Track geometry for trajectory mode.
  double joint_to_track{1.0}; // [m/rad] for revolute, [m/m] for prismatic
  double length{0.0};         // [m]
  double end_position{0.0};   // [rad] or [m]

  // Cached world-frame kinematics at q=0.
  gz::math::Pose3d joint_pose_world{gz::math::Pose3d::Zero};
  gz::math::Pose3d child_pose_world{gz::math::Pose3d::Zero};
  gz::math::Vector3d axis_world{1.0, 0.0, 0.0};

  std::vector< std::vector< gz::sim::Entity > > variant_visual_entities;
  std::vector< std::vector< gz::sim::Entity > > variant_collision_entities;
};

class ContinuousTrackSystem : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr< const sdf::Element > &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_event_mgr) override {
    this->ResetState();

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
    if (!this->LoadSprocket(plugin_sdf, _ecm)) {
      return;
    }

    if (plugin_sdf->HasElement("trajectory")) {
      this->use_trajectory_mode_ = true;
      this->LoadTrajectorySegments(plugin_sdf->GetElement("trajectory"), _ecm);
      if (this->segments_.empty()) {
        return;
      }
      this->LoadPattern(plugin_sdf);
      if (this->pattern_.enabled) {
        if (!this->BuildPatternGeometry(_ecm, _event_mgr)) {
          return;
        }

        const auto sprocket_pos = this->ReadSprocketPosition(_ecm);
        if (sprocket_pos) {
          this->pattern_.active_variant = this->CalcVariantId(*sprocket_pos);
        } else {
          this->pattern_.active_variant = 0;
        }
        this->ApplyVariant(this->pattern_.active_variant, _ecm);
      }
    } else if (plugin_sdf->HasElement("track")) {
      this->use_trajectory_mode_ = false;
      this->LoadTrackSegments(plugin_sdf->GetElement("track"), _ecm);
      if (this->segments_.empty()) {
        return;
      }
    } else {
      gzerr << "[ContinuousTrackSystem] Missing <track> or <trajectory> element." << std::endl;
      return;
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

    std::optional< double > sprocket_pos = this->ReadSprocketPosition(_ecm);
    if (this->pattern_.enabled && sprocket_pos) {
      const std::size_t next_variant = this->CalcVariantId(*sprocket_pos);
      if (next_variant != this->pattern_.active_variant) {
        this->ApplyVariant(next_variant, _ecm);
        this->pattern_.active_variant = next_variant;
      }
    }

    for (const auto &segment : this->segments_) {
      gz::sim::Joint segment_joint(segment.joint_entity);

      if (this->use_trajectory_mode_ && this->pattern_.enabled && sprocket_pos) {
        const double centered = this->NormalizeCentered(
            *sprocket_pos, this->pattern_.len_per_element);
        segment_joint.ResetPosition(_ecm, {centered / segment.joint_to_track});
      } else if (segment.reset_position) {
        segment_joint.ResetPosition(_ecm, {0.0});
      }

      segment_joint.SetVelocity(_ecm, {sprocket_vel * segment.velocity_scale});
    }
  }

private:
  void ResetState() {
    this->configured_ = false;
    this->use_trajectory_mode_ = false;
    this->sprocket_pitch_diameter_ = 0.0;
    this->sprocket_joint_entity_ = gz::sim::kNullEntity;
    this->segments_.clear();
    this->pattern_ = PatternConfig{};
  }

  bool LoadSprocket(const sdf::ElementPtr &_plugin_sdf, gz::sim::EntityComponentManager &_ecm) {
    if (!_plugin_sdf->HasElement("sprocket")) {
      gzerr << "[ContinuousTrackSystem] Missing required <sprocket> element." << std::endl;
      return false;
    }

    auto sprocket_elem = _plugin_sdf->GetElement("sprocket");
    if (!sprocket_elem->HasElement("joint") || !sprocket_elem->HasElement("pitch_diameter")) {
      gzerr << "[ContinuousTrackSystem] <sprocket> must contain <joint> and "
            << "<pitch_diameter>." << std::endl;
      return false;
    }

    const auto sprocket_joint_name = sprocket_elem->Get< std::string >("joint");
    this->sprocket_pitch_diameter_ = sprocket_elem->Get< double >("pitch_diameter");
    if (this->sprocket_pitch_diameter_ <= 0.0) {
      gzerr << "[ContinuousTrackSystem] <sprocket><pitch_diameter> must be positive."
            << std::endl;
      return false;
    }

    this->sprocket_joint_entity_ = this->model_.JointByName(_ecm, sprocket_joint_name);
    if (this->sprocket_joint_entity_ == gz::sim::kNullEntity) {
      gzerr << "[ContinuousTrackSystem] Joint not found: " << sprocket_joint_name << std::endl;
      return false;
    }

    gz::sim::Joint sprocket_joint(this->sprocket_joint_entity_);
    sprocket_joint.EnableVelocityCheck(_ecm, true);
    sprocket_joint.EnablePositionCheck(_ecm, true);
    return true;
  }

  void LoadTrackSegments(const sdf::ElementPtr &_track_elem, gz::sim::EntityComponentManager &_ecm) {
    if (!_track_elem->HasElement("segment")) {
      gzerr << "[ContinuousTrackSystem] <track> requires at least one <segment>." << std::endl;
      return;
    }

    for (auto segment_elem = _track_elem->GetElement("segment"); segment_elem;
         segment_elem = segment_elem->GetNextElement("segment")) {
      SegmentConfig segment;
      if (this->LoadSegmentCommon(segment_elem, _ecm, segment) &&
          this->FinalizeTrackSegment(segment_elem, _ecm, segment)) {
        this->segments_.push_back(segment);
      }
    }
  }

  void LoadTrajectorySegments(const sdf::ElementPtr &_traj_elem,
                              gz::sim::EntityComponentManager &_ecm) {
    if (!_traj_elem->HasElement("segment")) {
      gzerr << "[ContinuousTrackSystem] <trajectory> requires at least one <segment>."
            << std::endl;
      return;
    }

    this->pattern_.perimeter = 0.0;
    for (auto segment_elem = _traj_elem->GetElement("segment"); segment_elem;
         segment_elem = segment_elem->GetNextElement("segment")) {
      SegmentConfig segment;
      if (!this->LoadSegmentCommon(segment_elem, _ecm, segment)) {
        continue;
      }
      if (!this->FinalizeTrajectorySegment(segment_elem, _ecm, segment)) {
        continue;
      }

      this->pattern_.perimeter += segment.length;
      this->segments_.push_back(segment);
    }
  }

  bool LoadSegmentCommon(const sdf::ElementPtr &_segment_elem,
                         gz::sim::EntityComponentManager &_ecm,
                         SegmentConfig &_segment) {
    if (!_segment_elem->HasElement("joint")) {
      gzerr << "[ContinuousTrackSystem] <segment> is missing required <joint>." << std::endl;
      return false;
    }

    const auto segment_joint_name = _segment_elem->Get< std::string >("joint");
    _segment.joint_entity = this->model_.JointByName(_ecm, segment_joint_name);
    if (_segment.joint_entity == gz::sim::kNullEntity) {
      gzerr << "[ContinuousTrackSystem] Joint not found: " << segment_joint_name << std::endl;
      return false;
    }

    gz::sim::Joint segment_joint(_segment.joint_entity);
    const auto segment_type = segment_joint.Type(_ecm);
    if (!segment_type) {
      gzerr << "[ContinuousTrackSystem] Could not resolve joint type for: " << segment_joint_name
            << std::endl;
      return false;
    }
    _segment.type = *segment_type;

    _segment.child_link_entity = this->ResolveChildLinkEntity(segment_joint, _ecm);
    if (_segment.child_link_entity == gz::sim::kNullEntity) {
      gzerr << "[ContinuousTrackSystem] Failed to resolve child link entity for joint: "
            << segment_joint_name << std::endl;
      return false;
    }
    return true;
  }

  bool FinalizeTrackSegment(const sdf::ElementPtr &_segment_elem,
                            gz::sim::EntityComponentManager &_ecm,
                            SegmentConfig &_segment) const {
    if (_segment.type == sdf::JointType::REVOLUTE ||
        _segment.type == sdf::JointType::CONTINUOUS) {
      double segment_pitch_diameter = this->sprocket_pitch_diameter_;
      if (_segment_elem->HasElement("pitch_diameter")) {
        segment_pitch_diameter = _segment_elem->Get< double >("pitch_diameter");
      }
      if (segment_pitch_diameter <= 0.0) {
        gzerr << "[ContinuousTrackSystem] Segment pitch diameter must be positive." << std::endl;
        return false;
      }
      _segment.velocity_scale = this->sprocket_pitch_diameter_ / segment_pitch_diameter;
    } else if (_segment.type == sdf::JointType::PRISMATIC) {
      _segment.velocity_scale = this->sprocket_pitch_diameter_ / 2.0;
      _segment.reset_position = true;
      gz::sim::Joint(_segment.joint_entity).EnablePositionCheck(_ecm, true);
    } else {
      gzerr << "[ContinuousTrackSystem] Joint type must be revolute/continuous/prismatic."
            << std::endl;
      return false;
    }
    return true;
  }

  bool FinalizeTrajectorySegment(const sdf::ElementPtr &_segment_elem,
                                 gz::sim::EntityComponentManager &_ecm,
                                 SegmentConfig &_segment) const {
    if (!_segment_elem->HasElement("end_position")) {
      gzerr << "[ContinuousTrackSystem] <trajectory><segment> requires <end_position>."
            << std::endl;
      return false;
    }
    _segment.end_position = _segment_elem->Get< double >("end_position");
    if (_segment.end_position <= 0.0) {
      gzerr << "[ContinuousTrackSystem] <end_position> must be positive." << std::endl;
      return false;
    }

    if (_segment.type == sdf::JointType::REVOLUTE ||
        _segment.type == sdf::JointType::CONTINUOUS) {
      if (!this->FillRevoluteGeometry(_segment, _ecm)) {
        return false;
      }
      _segment.length = _segment.joint_to_track * _segment.end_position;
    } else if (_segment.type == sdf::JointType::PRISMATIC) {
      if (!this->FillPrismaticGeometry(_segment, _ecm)) {
        return false;
      }
      _segment.length = _segment.end_position;
      _segment.reset_position = true;
      gz::sim::Joint(_segment.joint_entity).EnablePositionCheck(_ecm, true);
    } else {
      gzerr << "[ContinuousTrackSystem] Joint type must be revolute/continuous/prismatic."
            << std::endl;
      return false;
    }

    if (_segment.joint_to_track <= 0.0 || _segment.length <= 0.0) {
      gzerr << "[ContinuousTrackSystem] Invalid trajectory segment geometry." << std::endl;
      return false;
    }

    _segment.velocity_scale =
        (this->sprocket_pitch_diameter_ / 2.0) / _segment.joint_to_track;
    return true;
  }

  bool FillRevoluteGeometry(SegmentConfig &_segment,
                            gz::sim::EntityComponentManager &_ecm) const {
    _segment.joint_pose_world = gz::sim::worldPose(_segment.joint_entity, _ecm);
    _segment.child_pose_world = gz::sim::worldPose(_segment.child_link_entity, _ecm);

    gz::sim::Joint joint(_segment.joint_entity);
    const auto axis_vec = joint.Axis(_ecm);
    if (!axis_vec || axis_vec->empty()) {
      gzerr << "[ContinuousTrackSystem] Missing joint axis for revolute segment." << std::endl;
      return false;
    }

    auto axis = axis_vec->at(0).Xyz();
    if (axis.Length() < 1e-8) {
      gzerr << "[ContinuousTrackSystem] Revolute segment axis is zero-length." << std::endl;
      return false;
    }

    _segment.axis_world = _segment.joint_pose_world.Rot().RotateVector(axis);
    _segment.axis_world.Normalize();
    const auto diff = _segment.child_pose_world.Pos() - _segment.joint_pose_world.Pos();
    _segment.joint_to_track = diff.Cross(_segment.axis_world).Length();
    if (_segment.joint_to_track <= 1e-8) {
      gzerr << "[ContinuousTrackSystem] Revolute segment radius is too small." << std::endl;
      return false;
    }
    return true;
  }

  bool FillPrismaticGeometry(SegmentConfig &_segment,
                             gz::sim::EntityComponentManager &_ecm) const {
    _segment.joint_pose_world = gz::sim::worldPose(_segment.joint_entity, _ecm);
    _segment.child_pose_world = gz::sim::worldPose(_segment.child_link_entity, _ecm);

    gz::sim::Joint joint(_segment.joint_entity);
    const auto axis_vec = joint.Axis(_ecm);
    if (!axis_vec || axis_vec->empty()) {
      gzerr << "[ContinuousTrackSystem] Missing joint axis for prismatic segment." << std::endl;
      return false;
    }

    auto axis = axis_vec->at(0).Xyz();
    if (axis.Length() < 1e-8) {
      gzerr << "[ContinuousTrackSystem] Prismatic segment axis is zero-length." << std::endl;
      return false;
    }

    _segment.axis_world = _segment.joint_pose_world.Rot().RotateVector(axis);
    _segment.axis_world.Normalize();
    _segment.joint_to_track = 1.0;
    return true;
  }

  gz::sim::Entity ResolveChildLinkEntity(const gz::sim::Joint &_joint,
                                         const gz::sim::EntityComponentManager &_ecm) const {
    const auto child_name = _joint.ChildLinkName(_ecm);
    if (!child_name) {
      return gz::sim::kNullEntity;
    }

    auto candidates =
        gz::sim::entitiesFromScopedName(*child_name, _ecm, this->model_.Entity(), "::");
    if (candidates.empty()) {
      candidates = gz::sim::entitiesFromScopedName(*child_name, _ecm, gz::sim::kNullEntity, "::");
    }
    if (candidates.empty()) {
      return gz::sim::kNullEntity;
    }
    for (const auto entity : candidates) {
      if (_ecm.EntityHasComponentType(entity, gz::sim::components::Link::typeId)) {
        return entity;
      }
    }
    return *candidates.begin();
  }

  void LoadPattern(const sdf::ElementPtr &_plugin_sdf) {
    if (!_plugin_sdf->HasElement("pattern")) {
      this->pattern_.enabled = false;
      return;
    }

    auto pattern_elem = _plugin_sdf->GetElement("pattern");
    if (!pattern_elem->HasElement("elements_per_round")) {
      gzerr << "[ContinuousTrackSystem] <pattern> requires <elements_per_round>." << std::endl;
      return;
    }
    this->pattern_.elements_per_round = pattern_elem->Get< std::size_t >("elements_per_round");
    if (this->pattern_.elements_per_round == 0) {
      gzerr << "[ContinuousTrackSystem] <elements_per_round> must be positive." << std::endl;
      return;
    }

    if (!pattern_elem->HasElement("element")) {
      gzerr << "[ContinuousTrackSystem] <pattern> requires at least one <element>." << std::endl;
      return;
    }

    this->pattern_.elements.clear();
    for (auto elem = pattern_elem->GetElement("element"); elem;
         elem = elem->GetNextElement("element")) {
      PatternElementConfig out_elem;

      if (elem->HasElement("visual")) {
        for (auto visual_elem = elem->GetElement("visual"); visual_elem;
             visual_elem = visual_elem->GetNextElement("visual")) {
          sdf::Visual visual;
          const auto errors = visual.Load(visual_elem->Clone());
          if (errors.empty()) {
            out_elem.visuals.push_back(visual);
          } else {
            gzwarn << "[ContinuousTrackSystem] Failed to parse one <visual> in <pattern>."
                   << std::endl;
          }
        }
      }

      if (elem->HasElement("collision")) {
        for (auto collision_elem = elem->GetElement("collision"); collision_elem;
             collision_elem = collision_elem->GetNextElement("collision")) {
          sdf::Collision collision;
          const auto errors = collision.Load(collision_elem->Clone());
          if (errors.empty() && this->NormalizeCollision(collision)) {
            out_elem.collisions.push_back(collision);
          } else {
            gzwarn << "[ContinuousTrackSystem] Failed to parse one <collision> in <pattern>."
                   << std::endl;
          }
        }
      }

      this->pattern_.elements.push_back(out_elem);
    }

    this->pattern_.enabled =
        !this->pattern_.elements.empty() &&
        this->pattern_.elements_per_round > 0 &&
        this->pattern_.perimeter > 0.0;
    if (!this->pattern_.enabled) {
      gzwarn << "[ContinuousTrackSystem] <pattern> exists but it is empty/invalid; ignored."
             << std::endl;
      return;
    }

    this->pattern_.len_per_element =
        this->pattern_.perimeter / static_cast< double >(this->pattern_.elements_per_round);
  }

  bool BuildPatternGeometry(gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_event_mgr) {
    if (!this->pattern_.enabled) {
      return true;
    }
    if (this->pattern_.len_per_element <= 0.0) {
      gzerr << "[ContinuousTrackSystem] Invalid pattern step length." << std::endl;
      return false;
    }

    gz::sim::SdfEntityCreator creator(_ecm, _event_mgr);
    const std::size_t variant_count = this->pattern_.elements.size();
    const double len_step = this->pattern_.len_per_element;

    for (auto &segment : this->segments_) {
      segment.variant_visual_entities.assign(variant_count, {});
      segment.variant_collision_entities.assign(variant_count, {});
    }

    for (std::size_t variant_id = 0; variant_id < variant_count; ++variant_id) {
      double len_left = 0.0;
      double len_traveled = 0.0;
      std::size_t elem_id = variant_count - 1 - variant_id;
      std::size_t elem_count = 0;

      for (std::size_t segm_id = 0; segm_id < this->segments_.size(); ++segm_id) {
        auto &segment = this->segments_[segm_id];
        len_left += segment.length;
        double seg_len_traveled = len_traveled;

        while (len_left >= 0.0 && elem_count < this->pattern_.elements_per_round) {
          const auto &element = this->pattern_.elements[elem_id];
          const auto base_pose = this->ComputeSegmentPoseOffset(segment, seg_len_traveled);

          for (std::size_t vis_id = 0; vis_id < element.visuals.size(); ++vis_id) {
            sdf::Visual visual = element.visuals[vis_id];
            visual.SetName("ct_variant" + std::to_string(variant_id) + "_elem" +
                           std::to_string(elem_count) + "_visual" + std::to_string(vis_id));
            visual.SetRawPose(visual.RawPose() + base_pose);

            const auto vis_entity = creator.CreateEntities(&visual);
            creator.SetParent(vis_entity, segment.child_link_entity);
            segment.variant_visual_entities[variant_id].push_back(vis_entity);

            _ecm.SetComponentData< gz::sim::components::Transparency >(vis_entity, 1.0f);
          }

          for (std::size_t col_id = 0; col_id < element.collisions.size(); ++col_id) {
            sdf::Collision collision = element.collisions[col_id];
            collision.SetName("ct_variant" + std::to_string(variant_id) + "_elem" +
                              std::to_string(elem_count) + "_collision" + std::to_string(col_id));
            collision.SetRawPose(collision.RawPose() + base_pose);
            this->SetCollisionBitmask(collision, 0x0000);
            if (!this->NormalizeCollision(collision)) {
              gzwarn << "[ContinuousTrackSystem] Skip invalid pattern collision after "
                        "bitmask update."
                     << std::endl;
              continue;
            }

            const auto col_entity = creator.CreateEntities(&collision);
            creator.SetParent(col_entity, segment.child_link_entity);
            segment.variant_collision_entities[variant_id].push_back(col_entity);
          }

          len_left -= len_step;
          seg_len_traveled += len_step;
          elem_id = (elem_id + 1) % variant_count;
          ++elem_count;
        }
        len_traveled = seg_len_traveled - segment.length;
      }
    }

    return true;
  }

  gz::math::Pose3d ComputeSegmentPoseOffset(const SegmentConfig &_segment,
                                            const double _track_distance) const {
    const double joint_pos = _track_distance / _segment.joint_to_track;

    if (_segment.type == sdf::JointType::PRISMATIC) {
      const auto world_delta = _segment.axis_world * joint_pos;
      const auto local_delta =
          _segment.child_pose_world.Rot().RotateVectorReverse(world_delta);
      return {local_delta, gz::math::Quaterniond::Identity};
    }

    const gz::math::Quaterniond world_rot(_segment.axis_world, joint_pos);
    const auto p0 = _segment.child_pose_world.Pos();
    const auto joint_p = _segment.joint_pose_world.Pos();

    const auto p = joint_p + world_rot.RotateVector(p0 - joint_p);
    const auto r = world_rot * _segment.child_pose_world.Rot();

    const auto local_pos =
        _segment.child_pose_world.Rot().RotateVectorReverse(p - p0);
    const auto local_rot = _segment.child_pose_world.Rot().Inverse() * r;
    return {local_pos, local_rot};
  }

  std::optional< double > ReadSprocketPosition(const gz::sim::EntityComponentManager &_ecm) const {
    const auto pos = gz::sim::Joint(this->sprocket_joint_entity_).Position(_ecm);
    if (!pos || pos->empty()) {
      return std::nullopt;
    }
    return pos->at(0) * (this->sprocket_pitch_diameter_ / 2.0);
  }

  double NormalizeCentered(const double _value, const double _period) const {
    if (_period <= 0.0) {
      return 0.0;
    }
    return _value - _period * std::floor(_value / _period) - _period / 2.0;
  }

  std::size_t CalcVariantId(const double _track_pos) const {
    if (!this->pattern_.enabled || this->pattern_.elements.empty() ||
        this->pattern_.len_per_element <= 0.0) {
      return 0;
    }

    const double len_per_variant_set =
        this->pattern_.len_per_element * this->pattern_.elements.size();
    const double pos =
        _track_pos - len_per_variant_set * std::floor(_track_pos / len_per_variant_set);
    const auto id = static_cast< std::size_t >(std::floor(pos / this->pattern_.len_per_element));
    return std::min(id, this->pattern_.elements.size() - 1);
  }

  void ApplyVariant(const std::size_t _variant, gz::sim::EntityComponentManager &_ecm) {
    for (auto &segment : this->segments_) {
      const std::size_t variant_count = segment.variant_visual_entities.size();
      for (std::size_t idx = 0; idx < variant_count; ++idx) {
        const bool active = (idx == _variant);

        for (const auto visual_entity : segment.variant_visual_entities[idx]) {
          _ecm.SetComponentData< gz::sim::components::Transparency >(
              visual_entity, active ? 0.0f : 1.0f);
        }

        for (const auto collision_entity : segment.variant_collision_entities[idx]) {
          const auto collision_comp =
              _ecm.Component< gz::sim::components::CollisionElement >(collision_entity);
          if (!collision_comp) {
            continue;
          }
          auto collision = collision_comp->Data();
          this->SetCollisionBitmask(collision, active ? 0xFFFF : 0x0000);
          _ecm.SetComponentData< gz::sim::components::CollisionElement >(collision_entity,
                                                                         collision);
        }
      }
    }
  }

  void SetCollisionBitmask(sdf::Collision &_collision, const uint16_t _mask) const {
    // Rebuild a minimal canonical surface/contact pair to avoid carrying
    // parser-specific element metadata that can break in downstream physics.
    sdf::Surface surface;
    sdf::Contact contact;
    contact.SetCollideBitmask(_mask);
    surface.SetContact(contact);
    _collision.SetSurface(surface);
  }

  bool NormalizeCollision(sdf::Collision &_collision) const {
    sdf::Errors errors;
    const auto canonical = _collision.ToElement(errors);
    if (!errors.empty() || !canonical) {
      return false;
    }

    sdf::Collision reloaded;
    const auto load_errors = reloaded.Load(canonical->Clone());
    if (!load_errors.empty()) {
      return false;
    }

    _collision = std::move(reloaded);
    return true;
  }

private:
  bool configured_{false};
  bool use_trajectory_mode_{false};
  double sprocket_pitch_diameter_{0.0};

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity sprocket_joint_entity_{gz::sim::kNullEntity};
  std::vector< SegmentConfig > segments_;
  PatternConfig pattern_{};
};

} // namespace gazebo_continuous_track

GZ_ADD_PLUGIN(gazebo_continuous_track::ContinuousTrackSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gazebo_continuous_track::ContinuousTrackSystem,
                    "gazebo_continuous_track::ContinuousTrackSystem")
