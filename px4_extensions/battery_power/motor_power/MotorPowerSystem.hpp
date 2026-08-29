/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 ****************************************************************************/

#pragma once

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <memory>
#include <string>
#include <vector>

namespace gz::sim::systems
{

class MotorPowerSystem :
	public System,
	public ISystemConfigure,
	public ISystemPreUpdate
{
public:
	void Configure(const Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       EntityComponentManager &_ecm,
		       EventManager &_eventMgr) override;

	void PreUpdate(const UpdateInfo &_info,
		       EntityComponentManager &_ecm) override;

private:
	bool InitializeJoints(EntityComponentManager &_ecm);

	Model _model{kNullEntity};
	std::vector<std::string> _joint_names;
	std::vector<Entity> _joint_entities;

	double _motor_constant{8.54858e-06};
	double _moment_constant{0.016};
	double _rotor_velocity_slowdown{10.0};
	double _motor_efficiency{0.82};
	double _last_power_w{0.0};
	double _static_power_w{8.0};
	double _capacity_ah{5.0};
	double _initial_charge_ah{5.0};
	double _charge_ah{5.0};
	double _ocv_full_v{16.8};
	double _ocv_delta_v{-2.4};
	double _internal_resistance_ohm{0.02};
	double _current_tau_s{0.5};
	double _voltage_v{16.8};
	double _current_a{0.0};
	double _soc{1.0};
	double _reset_charge_after_idle_s{3.0};
	double _idle_rotor_threshold_rad_s{1.0};
	double _rotor_idle_elapsed_s{0.0};

	std::string _battery_name{"linear_battery"};
	std::string _power_topic;
	gz::transport::Node _node;
	gz::transport::Node::Publisher _power_pub;
	gz::transport::Node::Publisher _battery_pub;
	double _publish_elapsed_s{0.0};
};

} // namespace gz::sim::systems
