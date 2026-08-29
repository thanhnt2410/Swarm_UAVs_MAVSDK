/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "MotorPowerSystem.hpp"

#include <gz/msgs/battery_state.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace gz;
using namespace sim;
using namespace systems;

void MotorPowerSystem::Configure(const Entity &_entity,
				 const std::shared_ptr<const sdf::Element> &_sdf,
				 EntityComponentManager &_ecm,
				 EventManager &)
{
	_model = Model(_entity);
	_joint_names = {"rotor_0_joint", "rotor_1_joint", "rotor_2_joint", "rotor_3_joint"};

	if (_sdf->HasElement("joint_name")) {
		_joint_names.clear();
		sdf::ElementConstPtr element = _sdf->FindElement("joint_name");

		while (element) {
			_joint_names.push_back(element->Get<std::string>());
			element = element->GetNextElement("joint_name");
		}
	}

	_sdf->Get("battery_name", _battery_name, _battery_name);
	_sdf->Get("motor_constant", _motor_constant, _motor_constant);
	_sdf->Get("moment_constant", _moment_constant, _moment_constant);
	_sdf->Get("rotor_velocity_slowdown", _rotor_velocity_slowdown, _rotor_velocity_slowdown);
	_sdf->Get("motor_efficiency", _motor_efficiency, _motor_efficiency);
	_sdf->Get("static_power", _static_power_w, _static_power_w);
	_sdf->Get("capacity", _capacity_ah, _capacity_ah);
	_sdf->Get("initial_charge", _initial_charge_ah, _capacity_ah);
	_sdf->Get("open_circuit_voltage_constant_coef", _ocv_full_v, _ocv_full_v);
	_sdf->Get("open_circuit_voltage_linear_coef", _ocv_delta_v, _ocv_delta_v);
	_sdf->Get("resistance", _internal_resistance_ohm, _internal_resistance_ohm);
	_sdf->Get("smooth_current_tau", _current_tau_s, _current_tau_s);
	_sdf->Get("reset_charge_after_idle_s", _reset_charge_after_idle_s, _reset_charge_after_idle_s);
	_sdf->Get("idle_rotor_threshold_rad_s", _idle_rotor_threshold_rad_s, _idle_rotor_threshold_rad_s);
	_motor_efficiency = std::clamp(_motor_efficiency, 0.01, 1.0);
	_capacity_ah = std::max(_capacity_ah, 0.001);
	_initial_charge_ah = std::clamp(_initial_charge_ah, 0.0, _capacity_ah);
	_charge_ah = _initial_charge_ah;
	_current_tau_s = std::max(_current_tau_s, 0.001);
	_reset_charge_after_idle_s = std::max(_reset_charge_after_idle_s, 0.0);
	_idle_rotor_threshold_rad_s = std::max(_idle_rotor_threshold_rad_s, 0.0);
	_soc = _charge_ah / _capacity_ah;
	_voltage_v = _ocv_full_v + _ocv_delta_v * (1.0 - _soc);

	_power_topic = "/model/" + _model.Name(_ecm) + "/motor_power";
	_power_pub = _node.Advertise<msgs::Double>(_power_topic);
	_battery_pub = _node.Advertise<msgs::BatteryState>(
			       "/model/" + _model.Name(_ecm) + "/battery/" + _battery_name + "/state");
}

bool MotorPowerSystem::InitializeJoints(EntityComponentManager &_ecm)
{
	if (_joint_entities.size() == _joint_names.size()) {
		return true;
	}

	_joint_entities.clear();

	for (const std::string &joint_name : _joint_names) {
		const Entity joint = _model.JointByName(_ecm, joint_name);

		if (joint == kNullEntity) {
			_joint_entities.clear();
			return false;
		}

		if (!_ecm.Component<components::JointVelocity>(joint)) {
			_ecm.CreateComponent(joint, components::JointVelocity());
		}

		_joint_entities.push_back(joint);
	}

	return true;
}

void MotorPowerSystem::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
	if (_info.paused || _info.dt <= std::chrono::steady_clock::duration::zero() || !InitializeJoints(_ecm)) {
		return;
	}

	double motor_power_w = 0.0;
	double max_rotor_velocity_rad_s = 0.0;

	for (const Entity joint : _joint_entities) {
		const auto *velocity = _ecm.Component<components::JointVelocity>(joint);

		if (!velocity || velocity->Data().empty()) {
			continue;
		}

		const double omega = std::abs(velocity->Data()[0]) * _rotor_velocity_slowdown;
		max_rotor_velocity_rad_s = std::max(max_rotor_velocity_rad_s, omega);
		const double thrust = _motor_constant * omega * omega;
		const double torque = _moment_constant * thrust;
		motor_power_w += std::abs(torque * omega) / _motor_efficiency;
	}

	_last_power_w = std::max(motor_power_w, 0.0);
	const double dt_s = std::chrono::duration<double>(_info.dt).count();
	if (max_rotor_velocity_rad_s <= _idle_rotor_threshold_rad_s) {
		_rotor_idle_elapsed_s += dt_s;
	} else {
		_rotor_idle_elapsed_s = 0.0;
	}

	if (_rotor_idle_elapsed_s >= _reset_charge_after_idle_s) {
		_charge_ah = _initial_charge_ah;
		_current_a = 0.0;
		_soc = _charge_ah / _capacity_ah;
		_voltage_v = _ocv_full_v + _ocv_delta_v * (1.0 - _soc);
	}

	const double total_power_w = _static_power_w + _last_power_w;
	const double raw_current_a = total_power_w / std::max(_voltage_v, 0.1);
	const double alpha = std::clamp(dt_s / (_current_tau_s + dt_s), 0.0, 1.0);
	_current_a += alpha * (raw_current_a - _current_a);
	_charge_ah = std::max(0.0, _charge_ah - _current_a * dt_s / 3600.0);
	_soc = _charge_ah / _capacity_ah;
	const double open_circuit_voltage = _ocv_full_v + _ocv_delta_v * (1.0 - _soc);
	_voltage_v = std::max(0.0, open_circuit_voltage - _internal_resistance_ohm * _current_a);
	_publish_elapsed_s += dt_s;

	if (_publish_elapsed_s >= 0.02) {
		msgs::BatteryState battery_message;
		battery_message.mutable_header()->mutable_stamp()->set_sec(
			static_cast<int64_t>(std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count()));
		battery_message.mutable_header()->mutable_stamp()->set_nsec(
			static_cast<int32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count() % 1000000000));
		battery_message.set_voltage(_voltage_v);
		battery_message.set_current(_current_a);
		battery_message.set_charge(_charge_ah);
		battery_message.set_capacity(_capacity_ah);
		battery_message.set_percentage(_soc * 100.0);
		battery_message.set_power_supply_status(msgs::BatteryState::DISCHARGING);
		_battery_pub.Publish(battery_message);
	}

	if (_power_pub.Valid() && _publish_elapsed_s >= 0.1) {
		msgs::Double message;
		message.set_data(_last_power_w);
		_power_pub.Publish(message);
	}

	if (_publish_elapsed_s >= 0.1) {
		_publish_elapsed_s = 0.0;
	}
}

GZ_ADD_PLUGIN(MotorPowerSystem,
	      System,
	      MotorPowerSystem::ISystemConfigure,
	      MotorPowerSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(MotorPowerSystem, "gz::sim::systems::MotorPowerSystem")
