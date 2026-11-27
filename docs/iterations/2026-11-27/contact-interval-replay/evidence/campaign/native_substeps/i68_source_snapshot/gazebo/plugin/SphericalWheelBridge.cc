// Gazebo Harmonic (gz-sim 8) bridge for the active spherical wheel.
//
// The plugin is attached to the wheel model. It owns no controller: the
// unchanged Python controller computes the five drive commands and this
// plugin applies them, steps in lockstep with Python and reports the state
// and the solved wheel-ground contacts back.
//
// Contact model (matches contact.mode = analytic_clipped_floor in MuJoCo):
//   * shell_A_smooth, shell_B_smooth, belt_A_smooth, belt_B_smooth are four
//     concentric spheres of the wheel radius. A contact on one of them is
//     admitted only when its support direction lies on that surface's own
//     spherical patch, evaluated in the carrier frame exactly like the MuJoCo
//     collision filter (contact.py::_clipped_floor_filter). Any other contact
//     on a smooth sphere is switched off: its target normal separation speed
//     is set to -1000 m/s, so the LCP gives it zero impulse and zero friction.
//   * shell_X_rim_NNN capsules trace the 48-sided equatorial shell rims. They
//     are real geometry and are always admitted, so a support point inside the
//     equatorial gap rests on the rims as in MuJoCo.
//   * Every other collision of the model (fork arms, bridge, platform) is
//     admitted; touching the ground with them is a fall.
//   * A belt contact gets a target tangential surface speed equal to the
//     commanded belt speed along the carrier y axis (the ideal-servo belt of
//     sim.py, MuJoCo geom_surfacevel).
//   * Every admitted contact uses the ground friction coefficient Python sends
//     for the current control interval (MuJoCo: floor priority friction).
//
// The C functions at the bottom are the Python interface (loaded with ctypes
// from the same shared object gz-sim loaded, so they see the same state).

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/PhysicsCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/msgs/physics.pb.h>
#include <gz/sim/physics/Events.hh>

namespace asw
{
using namespace gz;
using namespace gz::sim;

// Collision tags shared with Python (spherical_wheel/gazebo/bridge.py).
enum Tag : int
{
  kShellASmooth = 0, kShellBSmooth = 1, kBeltASmooth = 2, kBeltBSmooth = 3,
  kShellARim = 4, kShellBRim = 5, kBody = 6
};

constexpr int kStateSize = 24;    // see asw_state()
constexpr int kContactStride = 16;  // see asw_contacts()
constexpr double kDisableNormalSpeed = -1000.0;
// A reset is applied during one step this short, so contacts are solved at
// the reset state while the state itself advances by a microsecond.
constexpr double kResetStep = 1e-6;
// DART's default maximum penetration-correction speed (m/s).
constexpr double kMaxErrorReductionVelocity = 1e-3;

struct ContactRecord
{
  int tag;
  std::array<double, 3> position, normal, force;
  double depth, friction;
  bool admitted;
};

struct Decision
{
  Entity c1, c2;
  math::Vector3d point;
  bool admitted;
  double friction;
};

struct Shared
{
  std::mutex mutex;
  bool ready = false;
  // Command held for the current control interval.
  std::array<double, 3> torque{0, 0, 0};
  std::array<double, 2> belt{0, 0};
  std::array<double, 6> wrench{0, 0, 0, 0, 0, 0};
  double friction = 0.7;
  // Reset request: fork pose (x y z qw qx qy qz), joints, base twist (world).
  bool resetPending = false;
  std::array<double, 7> resetPose{};
  std::array<double, 3> resetJointPosition{}, resetJointVelocity{};
  std::array<double, 3> resetLinear{}, resetAngular{};
  // Output.
  std::array<double, kStateSize> state{};
  std::vector<ContactRecord> contacts;
  std::uint64_t admitted = 0, rejected = 0, unmatched = 0;
  double resets = 0;
};

Shared &shared()
{
  static Shared instance;
  return instance;
}

class SphericalWheelBridge : public System, public ISystemConfigure,
                             public ISystemPreUpdate, public ISystemPostUpdate
{
public:
  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &_eventMgr) override
  {
    this->model = Model(_entity);
    this->radius = _sdf->Get<double>("sphere_radius", 0.12).first;
    this->gapHalf = _sdf->Get<double>("equatorial_gap_half", 0.01).first;
    this->polarRim = _sdf->Get<double>("polar_rim", this->radius * std::cos(15.46600995 * M_PI / 180)).first;
    this->forkCom = _sdf->Get<math::Vector3d>("fork_com", math::Vector3d(0, 0, 0)).first;
    this->allowance = _sdf->Get<double>("contact_error_allowance", 5e-5).first;
    this->connection = _eventMgr.Connect<events::CollectContactSurfaceProperties>(
      [this](const Entity &_c1, const Entity &_c2, const math::Vector3d &_point,
             const std::optional<math::Vector3d>, const std::optional<math::Vector3d> _normal,
             const std::optional<double>, const size_t,
             physics::SetContactPropertiesCallbackFeature::ContactSurfaceParams<events::Policy> &_params)
      { this->OnContact(_c1, _c2, _point, _normal, _params); });
    (void)_ecm;
    // A new world in the same process starts from a clean interface.
    auto &s = shared();
    std::lock_guard<std::mutex> lock(s.mutex);
    s.ready = false;
    s.torque = {0, 0, 0};
    s.belt = {0, 0};
    s.wrench = {0, 0, 0, 0, 0, 0};
    s.resetPending = false;
    s.contacts.clear();
    s.admitted = s.rejected = s.unmatched = 0;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (!this->initialized && !this->Initialize(_ecm))
      return;
    auto &s = shared();
    std::lock_guard<std::mutex> lock(s.mutex);
    this->decisions.clear();
    const double dt = std::chrono::duration<double>(_info.dt).count();
    this->lastStep = dt;
    this->resetting = false;
    if (this->clearVelocityCommands)
    {
      // gz-sim zeroes velocity commands after a step but keeps the components,
      // which would then pin the base at zero velocity on every later step.
      _ecm.RemoveComponent<components::LinearVelocityCmd>(this->fork.Entity());
      _ecm.RemoveComponent<components::AngularVelocityCmd>(this->fork.Entity());
      this->clearVelocityCommands = false;
    }
    if (s.resetPending)
    {
      if (dt > 2 * kResetStep)
      {
        // Shrink the next step first; hold the drives idle meanwhile.
        if (!this->shrinking)
        {
          this->nominalStep = dt;
          this->RequestStep(_ecm, kResetStep);
          this->shrinking = true;
        }
        this->UpdateKinematics(_ecm);
        this->belt = {0, 0};
        return;
      }
      const auto &p = s.resetPose;
      math::Pose3d pose(p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
      this->model.SetWorldPoseCmd(_ecm, pose);
      for (int i = 0; i < 3; ++i)
      {
        this->joints[i].ResetPosition(_ecm, {s.resetJointPosition[i]});
        this->joints[i].ResetVelocity(_ecm, {s.resetJointVelocity[i]});
      }
      // The physics system rotates a link velocity command into the world with
      // the model's Pose component, which still holds the pre-reset pose in
      // this iteration. The fork is the model's root link at its origin.
      const auto *modelPose = _ecm.Component<components::Pose>(this->model.Entity());
      const math::Quaterniond current = modelPose ? modelPose->Data().Rot() : pose.Rot();
      math::Vector3d lin(s.resetLinear[0], s.resetLinear[1], s.resetLinear[2]);
      math::Vector3d ang(s.resetAngular[0], s.resetAngular[1], s.resetAngular[2]);
      this->fork.SetLinearVelocity(_ecm, current.RotateVectorReverse(lin));
      this->fork.SetAngularVelocity(_ecm, current.RotateVectorReverse(ang));
      this->clearVelocityCommands = true;
      this->center = pose.Pos();
      this->carrierX = (pose.Rot() * math::Quaterniond(0, s.resetJointPosition[0], 0)).RotateVector(math::Vector3d::UnitX);
      this->carrierY = pose.Rot().RotateVector(math::Vector3d::UnitY);
      this->RequestStep(_ecm, this->nominalStep);
      this->shrinking = false;
      this->resetting = true;
      s.resetPending = false;
      s.resets += 1;
      this->belt = {0, 0};
      this->friction = s.friction;
      return;
    }
    this->UpdateKinematics(_ecm);
    this->belt = s.belt;
    this->friction = s.friction;
    for (int i = 0; i < 3; ++i)
      this->joints[i].SetForce(_ecm, {s.torque[i]});
    math::Vector3d force(s.wrench[0], s.wrench[1], s.wrench[2]);
    math::Vector3d torque(s.wrench[3], s.wrench[4], s.wrench[5]);
    if (force != math::Vector3d::Zero || torque != math::Vector3d::Zero)
      this->fork.AddWorldWrench(_ecm, force, torque, this->forkCom);
  }

  void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override
  {
    if (!this->initialized)
      return;
    auto &s = shared();
    std::lock_guard<std::mutex> lock(s.mutex);
    auto pose = this->fork.WorldPose(_ecm);
    auto lin = this->fork.WorldLinearVelocity(_ecm);
    auto ang = this->fork.WorldAngularVelocity(_ecm);
    std::array<double, kStateSize> &out = s.state;
    out.fill(std::nan(""));
    out[0] = std::chrono::duration<double>(_info.simTime).count();
    out[1] = static_cast<double>(_info.iterations);
    if (pose)
    {
      out[2] = pose->Pos().X(); out[3] = pose->Pos().Y(); out[4] = pose->Pos().Z();
      out[5] = pose->Rot().W(); out[6] = pose->Rot().X(); out[7] = pose->Rot().Y(); out[8] = pose->Rot().Z();
    }
    if (lin) { out[9] = lin->X(); out[10] = lin->Y(); out[11] = lin->Z(); }
    if (ang) { out[12] = ang->X(); out[13] = ang->Y(); out[14] = ang->Z(); }
    for (int i = 0; i < 3; ++i)
    {
      auto q = this->joints[i].Position(_ecm);
      auto v = this->joints[i].Velocity(_ecm);
      if (q && !q->empty()) out[15 + i] = (*q)[0];
      if (v && !v->empty()) out[18 + i] = (*v)[0];
    }
    out[21] = std::chrono::duration<double>(_info.dt).count();
    out[22] = s.resets;
    out[23] = s.resetPending ? 1.0 : 0.0;
    s.contacts.clear();
    for (const auto &[entity, tag] : this->tags)
    {
      auto *data = _ecm.Component<components::ContactSensorData>(entity);
      if (!data)
        continue;
      for (const auto &contact : data->Data().contact())
      {
        const bool first = contact.collision1().id() == entity;
        const Entity other = first ? contact.collision2().id() : contact.collision1().id();
        if (this->tags.count(other))
          continue;  // internal pair; never enabled
        for (int i = 0; i < contact.position_size(); ++i)
        {
          math::Vector3d point = msgs::Convert(contact.position(i));
          const Decision *decision = this->Find(contact.collision1().id(), contact.collision2().id(), point);
          bool admitted;
          double mu = this->friction;
          if (decision)
          {
            admitted = decision->admitted;
            mu = decision->friction;
          }
          else
          {
            ++s.unmatched;
            admitted = this->Admit(tag, point);
          }
          // Rejected contacts are reported too (flagged), so their forces can
          // be checked; they carry no load by construction.
          ContactRecord record;
          record.admitted = admitted;
          record.tag = tag;
          record.position = {point.X(), point.Y(), point.Z()};
          math::Vector3d normal = i < contact.normal_size() ? msgs::Convert(contact.normal(i)) : math::Vector3d::UnitZ;
          // Report the normal pointing from the ground into the wheel.
          if (normal.Dot(this->center - point) < 0)
            normal = -normal;
          record.normal = {normal.X(), normal.Y(), normal.Z()};
          math::Vector3d force = math::Vector3d::Zero;
          if (i < contact.wrench_size())
          {
            const auto &w = contact.wrench(i);
            force = msgs::Convert(first ? w.body_1_wrench().force() : w.body_2_wrench().force());
          }
          record.force = {force.X(), force.Y(), force.Z()};
          record.depth = i < contact.depth_size() ? contact.depth(i) : 0.0;
          record.friction = mu;
          s.contacts.push_back(record);
        }
      }
    }
  }

private:
  void RequestStep(EntityComponentManager &_ecm, double _step)
  {
    Entity world = _ecm.EntityByComponents(components::World());
    msgs::Physics msg;
    msg.set_max_step_size(_step);
    msg.set_real_time_factor(0.0);  // zero leaves the real-time factor unchanged
    auto *command = _ecm.Component<components::PhysicsCmd>(world);
    if (command)
      command->Data() = msg;
    else
      _ecm.CreateComponent(world, components::PhysicsCmd(msg));
  }

  bool Initialize(EntityComponentManager &_ecm)
  {
    const char *jointNames[3] = {"shaft", "shell_A_joint", "shell_B_joint"};
    for (int i = 0; i < 3; ++i)
    {
      Entity joint = this->model.JointByName(_ecm, jointNames[i]);
      if (joint == kNullEntity)
        return false;
      this->joints[i] = Joint(joint);
      this->joints[i].EnablePositionCheck(_ecm, true);
      this->joints[i].EnableVelocityCheck(_ecm, true);
    }
    Entity forkEntity = this->model.LinkByName(_ecm, "fork");
    if (forkEntity == kNullEntity)
      return false;
    this->fork = Link(forkEntity);
    this->fork.EnableVelocityChecks(_ecm, true);
    for (const char *name : {"fork", "carrier", "shell_A", "shell_B"})
    {
      Link link(this->model.LinkByName(_ecm, name));
      for (Entity collision : link.Collisions(_ecm))
      {
        auto *nameComponent = _ecm.Component<components::Name>(collision);
        const std::string n = nameComponent ? nameComponent->Data() : "";
        int tag = kBody;
        if (n == "shell_A_smooth") tag = kShellASmooth;
        else if (n == "shell_B_smooth") tag = kShellBSmooth;
        else if (n == "belt_A_smooth") tag = kBeltASmooth;
        else if (n == "belt_B_smooth") tag = kBeltBSmooth;
        else if (n.rfind("shell_A_rim", 0) == 0) tag = kShellARim;
        else if (n.rfind("shell_B_rim", 0) == 0) tag = kShellBRim;
        this->tags[collision] = tag;
        _ecm.CreateComponent(collision, components::EnableContactSurfaceCustomization(true));
        _ecm.CreateComponent(collision, components::ContactSensorData());
      }
    }
    if (this->tags.size() < 4)
      return false;
    this->UpdateKinematics(_ecm);
    this->initialized = true;
    std::lock_guard<std::mutex> lock(shared().mutex);
    shared().ready = true;
    gzmsg << "[SphericalWheelBridge] ready with " << this->tags.size() << " wheel collisions" << std::endl;
    return true;
  }

  void UpdateKinematics(const EntityComponentManager &_ecm)
  {
    auto pose = this->fork.WorldPose(_ecm);
    auto shaft = this->joints[0].Position(_ecm);
    if (!pose)
      return;
    const double q = (shaft && !shaft->empty()) ? (*shaft)[0] : 0.0;
    this->center = pose->Pos();
    this->carrierX = (pose->Rot() * math::Quaterniond(0, q, 0)).RotateVector(math::Vector3d::UnitX);
    this->carrierY = pose->Rot().RotateVector(math::Vector3d::UnitY);
  }

  // MuJoCo patch rule: sign * local_x of the ideal support point within
  // [lower, upper] of the surface's own patch.
  bool Admit(int _tag, const math::Vector3d &_point) const
  {
    if (_tag > kBeltBSmooth)
      return true;
    math::Vector3d direction = _point - this->center;
    const double length = direction.Length();
    if (length < 1e-12)
      return false;
    const double localX = this->radius * this->carrierX.Dot(direction / length);
    const double sign = (_tag == kShellASmooth || _tag == kBeltASmooth) ? 1.0 : -1.0;
    const bool shell = _tag == kShellASmooth || _tag == kShellBSmooth;
    const double lower = shell ? this->gapHalf : this->polarRim;
    const double upper = shell ? this->polarRim : this->radius;
    const double value = sign * localX;
    return lower - 1e-12 <= value && value <= upper + 1e-12;
  }

  void OnContact(const Entity &_c1, const Entity &_c2, const math::Vector3d &_point,
                 const std::optional<math::Vector3d> &_normal,
                 physics::SetContactPropertiesCallbackFeature::ContactSurfaceParams<events::Policy> &_params)
  {
    auto first = this->tags.find(_c1);
    auto second = this->tags.find(_c2);
    if (first == this->tags.end() && second == this->tags.end())
      return;
    const bool oursFirst = first != this->tags.end();
    const int tag = oursFirst ? first->second : second->second;
    const bool admitted = this->Admit(tag, _point);
    // DART's penetration allowance is one global value; every contact sets it.
    // With none, DART pushes a resting contact fully out, the next collision
    // check misses it and the wheel free-falls for a step: a micro-hop.
    _params.maxErrorAllowance = this->allowance;
    // During the microsecond reset step DART's penetration correction would
    // report correction impulse / 1 us as contact force; the reading must be
    // the static load. Otherwise DART's default correction speed applies.
    _params.maxErrorReductionVelocity = this->resetting ? 0.0 : kMaxErrorReductionVelocity;
    auto &s = shared();
    if (admitted) ++s.admitted; else ++s.rejected;
    this->decisions.push_back({_c1, _c2, _point, admitted, this->friction});
    if (!admitted)
    {
      _params.frictionCoeff = 0.0;
      _params.secondaryFrictionCoeff = 0.0;
      // DART order: (normal, first friction direction, second friction direction).
      _params.contactSurfaceMotionVelocity = Eigen::Vector3d(kDisableNormalSpeed, 0.0, 0.0);
      return;
    }
    _params.frictionCoeff = this->friction;
    _params.secondaryFrictionCoeff = this->friction;
    if (tag == kBeltASmooth || tag == kBeltBSmooth)
    {
      math::Vector3d normal = _normal ? *_normal : (this->center - _point).Normalized();
      normal.Normalize();
      math::Vector3d along = this->carrierY - normal * normal.Dot(this->carrierY);
      const double size = along.Length();
      if (size > 1e-9)
      {
        const double speed = this->belt[tag == kBeltASmooth ? 0 : 1];
        // No-slip with a belt moving at +speed along carrier y relative to the
        // carrier means the carrier material point moves at -speed*along over
        // the ground. DART's target is (body1 - body2) along the direction.
        const double target = (oursFirst ? -1.0 : 1.0) * speed * size;
        along /= size;
        _params.firstFrictionalDirection = Eigen::Vector3d(along.X(), along.Y(), along.Z());
        _params.contactSurfaceMotionVelocity = Eigen::Vector3d(0.0, target, 0.0);
      }
    }
  }

  const Decision *Find(Entity _c1, Entity _c2, const math::Vector3d &_point) const
  {
    for (const auto &d : this->decisions)
      if (d.c1 == _c1 && d.c2 == _c2 && d.point.Equal(_point, 1e-12))
        return &d;
    for (const auto &d : this->decisions)
      if (d.c1 == _c2 && d.c2 == _c1 && d.point.Equal(_point, 1e-12))
        return &d;
    return nullptr;
  }

  Model model{kNullEntity};
  Link fork{kNullEntity};
  std::array<Joint, 3> joints{Joint(kNullEntity), Joint(kNullEntity), Joint(kNullEntity)};
  std::map<Entity, int> tags;
  std::vector<Decision> decisions;
  common::ConnectionPtr connection;
  bool initialized = false;
  bool shrinking = false;
  bool clearVelocityCommands = false;
  bool resetting = false;
  double nominalStep = 0.001, lastStep = 0.0;
  double radius = 0.12, gapHalf = 0.01, polarRim = 0.1156546;
  math::Vector3d forkCom, center, carrierX{1, 0, 0}, carrierY{0, 1, 0};
  std::array<double, 2> belt{0, 0};
  double friction = 0.7;
  double allowance = 5e-5;
};
}  // namespace asw

GZ_ADD_PLUGIN(asw::SphericalWheelBridge, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)

// ---------------------------------------------------------------------------
// Python interface (ctypes). All arrays are caller-owned doubles.
extern "C"
{
int asw_ready()
{
  std::lock_guard<std::mutex> lock(asw::shared().mutex);
  return asw::shared().ready ? 1 : 0;
}

int asw_state_size() { return asw::kStateSize; }
int asw_contact_stride() { return asw::kContactStride; }

// tau[3] N m, belt[2] m/s, wrench[6] world N and N m at the fork COM, friction.
void asw_set_command(const double *_torque, const double *_belt, const double *_wrench, double _friction)
{
  auto &s = asw::shared();
  std::lock_guard<std::mutex> lock(s.mutex);
  for (int i = 0; i < 3; ++i) s.torque[i] = _torque[i];
  for (int i = 0; i < 2; ++i) s.belt[i] = _belt[i];
  for (int i = 0; i < 6; ++i) s.wrench[i] = _wrench[i];
  s.friction = _friction;
}

// pose[7] = x y z qw qx qy qz of the fork frame; joints[3]; joint_speeds[3];
// linear[3], angular[3] world-frame velocity of the fork frame origin.
void asw_request_reset(const double *_pose, const double *_joints, const double *_speeds,
                       const double *_linear, const double *_angular)
{
  auto &s = asw::shared();
  std::lock_guard<std::mutex> lock(s.mutex);
  for (int i = 0; i < 7; ++i) s.resetPose[i] = _pose[i];
  for (int i = 0; i < 3; ++i)
  {
    s.resetJointPosition[i] = _joints[i];
    s.resetJointVelocity[i] = _speeds[i];
    s.resetLinear[i] = _linear[i];
    s.resetAngular[i] = _angular[i];
  }
  s.resetPending = true;
}

// [time, iterations, x y z, qw qx qy qz, vx vy vz (world, frame origin),
//  wx wy wz (world), q_shaft q_A q_B, dq_shaft dq_A dq_B, step length,
//  resets applied, reset pending]
int asw_state(double *_out)
{
  auto &s = asw::shared();
  std::lock_guard<std::mutex> lock(s.mutex);
  std::memcpy(_out, s.state.data(), sizeof(double) * asw::kStateSize);
  return asw::kStateSize;
}

// Each contact: [tag, px py pz, nx ny nz (ground -> wheel), fx fy fz (on
//  the wheel, world), depth, friction, admitted, 0 0 0].
int asw_contacts(double *_out, int _capacity)
{
  auto &s = asw::shared();
  std::lock_guard<std::mutex> lock(s.mutex);
  const int count = static_cast<int>(s.contacts.size());
  const int written = count < _capacity ? count : _capacity;
  for (int i = 0; i < written; ++i)
  {
    const auto &c = s.contacts[i];
    double *row = _out + i * asw::kContactStride;
    std::memset(row, 0, sizeof(double) * asw::kContactStride);
    row[0] = c.tag;
    for (int k = 0; k < 3; ++k)
    {
      row[1 + k] = c.position[k];
      row[4 + k] = c.normal[k];
      row[7 + k] = c.force[k];
    }
    row[10] = c.depth;
    row[11] = c.friction;
    row[12] = c.admitted ? 1.0 : 0.0;
  }
  return count;
}

// admitted, rejected, unmatched contact decisions since load.
void asw_counters(double *_out)
{
  auto &s = asw::shared();
  std::lock_guard<std::mutex> lock(s.mutex);
  _out[0] = static_cast<double>(s.admitted);
  _out[1] = static_cast<double>(s.rejected);
  _out[2] = static_cast<double>(s.unmatched);
}
}
