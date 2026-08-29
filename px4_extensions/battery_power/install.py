#!/usr/bin/env python3
"""Install the repository-owned Gazebo battery extension into a PX4 checkout."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
from pathlib import Path
from xml.sax.saxutils import escape

import yaml


SUPPORTED_PX4_REVISION = "44c128aade5984f4824225145ab8b58000fcd6dd"


def replace_once(path: Path, old: str, new: str, installed_marker: str) -> None:
    text = path.read_text()
    if installed_marker in text:
        return
    if text.count(old) != 1:
        raise RuntimeError(f"Cannot find a unique patch anchor in {path}: {old!r}")
    path.write_text(text.replace(old, new, 1))


def install_plugin_sources(px4_dir: Path, extension_dir: Path) -> None:
    source = extension_dir / "motor_power"
    destination = px4_dir / "src/modules/simulation/gz_plugins/motor_power"
    destination.mkdir(parents=True, exist_ok=True)
    for source_file in source.iterdir():
        if source_file.is_file():
            shutil.copy2(source_file, destination / source_file.name)


def patch_plugin_build(px4_dir: Path) -> None:
    path = px4_dir / "src/modules/simulation/gz_plugins/CMakeLists.txt"
    replace_once(
        path,
        "    add_subdirectory(motor_failure)\n",
        "    add_subdirectory(motor_failure)\n    add_subdirectory(motor_power)\n",
        "add_subdirectory(motor_power)",
    )

    text = path.read_text()
    if "MotorPowerSystem" not in text:
        marker = "MotorFailurePlugin AirSpeedPlugin"
        if marker not in text:
            raise RuntimeError(f"Cannot find plugin target list in {path}")
        path.write_text(text.replace(marker, "MotorFailurePlugin MotorPowerSystem AirSpeedPlugin"))


def patch_bridge_build(px4_dir: Path) -> None:
    path = px4_dir / "src/modules/simulation/gz_bridge/CMakeLists.txt"
    replace_once(
        path,
        "\t\tDEPENDS\n\t\t\tmixer_module\n",
        "\t\tDEPENDS\n\t\t\tbattery\n\t\t\tmixer_module\n",
        "\t\t\tbattery\n",
    )


def patch_bridge_header(px4_dir: Path) -> None:
    path = px4_dir / "src/modules/simulation/gz_bridge/GZBridge.hpp"
    replacements = (
        (
            "#include <lib/drivers/device/Device.hpp>\n",
            "#include <lib/drivers/device/Device.hpp>\n#include <lib/battery/battery.h>\n",
            "#include <lib/battery/battery.h>",
        ),
        (
            "#include <gz/msgs/scene.pb.h>\n",
            "#include <gz/msgs/scene.pb.h>\n#include <gz/msgs/battery_state.pb.h>\n",
            "#include <gz/msgs/battery_state.pb.h>",
        ),
        (
            "\tbool subscribeOpticalFlow(bool required);\n",
            "\tbool subscribeOpticalFlow(bool required);\n\tbool subscribeBattery(bool required);\n",
            "bool subscribeBattery(bool required);",
        ),
        (
            "\tvoid magnetometerCallback(const gz::msgs::Magnetometer &msg);\n",
            "\tvoid magnetometerCallback(const gz::msgs::Magnetometer &msg);\n"
            "\tvoid batteryCallback(const gz::msgs::BatteryState &msg);\n",
            "void batteryCallback(const gz::msgs::BatteryState &msg);",
        ),
        (
            "\tuORB::PublicationMulti<sensor_optical_flow_s> _optical_flow_pub{ORB_ID(sensor_optical_flow)};\n",
            "\tuORB::PublicationMulti<sensor_optical_flow_s> _optical_flow_pub{ORB_ID(sensor_optical_flow)};\n\n"
            "\tstatic constexpr int BATTERY_SAMPLE_INTERVAL_US = 100000;\n"
            "\tBattery _battery;\n",
            "BATTERY_SAMPLE_INTERVAL_US = 100000",
        ),
    )
    for old, new, marker in replacements:
        replace_once(path, old, new, marker)


def patch_bridge_source(px4_dir: Path) -> None:
    path = px4_dir / "src/modules/simulation/gz_bridge/GZBridge.cpp"
    replace_once(
        path,
        "\tScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),\n\t_world_name(world),\n",
        "\tScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),\n"
        "\t_battery(1, this, BATTERY_SAMPLE_INTERVAL_US, battery_status_s::SOURCE_POWER_MODULE),\n"
        "\t_world_name(world),\n",
        "_battery(1, this, BATTERY_SAMPLE_INTERVAL_US",
    )
    replace_once(
        path,
        "\tif (_sim_gz_en_lidar.get()) {\n\t\tif (!subscribeDistanceSensor(false)) {\n",
        "\tif (!subscribeBattery(false)) {\n\t\treturn PX4_ERROR;\n\t}\n\n"
        "\tif (_sim_gz_en_lidar.get()) {\n\t\tif (!subscribeDistanceSensor(false)) {\n",
        "if (!subscribeBattery(false))",
    )

    battery_methods = """bool GZBridge::subscribeBattery(bool required)
{
\tconst std::string battery_topic = \"/model/\" + _model_name + \"/battery/linear_battery/state\";

\tif (!_node.Subscribe(battery_topic, &GZBridge::batteryCallback, this)) {
\t\tPX4_WARN(\"failed to subscribe to %s\", battery_topic.c_str());
\t\treturn required ? false : true;
\t}

\treturn true;
}

void GZBridge::batteryCallback(const gz::msgs::BatteryState &msg)
{
\tif (hrt_absolute_time() == 0 || !PX4_ISFINITE(msg.voltage()) || !PX4_ISFINITE(msg.current())) {
\t\treturn;
\t}

\t_battery.setConnected(msg.voltage() > 0.0);
\t_battery.updateVoltage(static_cast<float>(msg.voltage()));
\t_battery.updateCurrent(math::max(static_cast<float>(msg.current()), 0.0f));
\t_battery.setStateOfCharge(math::constrain(static_cast<float>(msg.percentage()) / 100.0f, 0.0f, 1.0f));
\t_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
}

"""
    replace_once(
        path,
        "bool GZBridge::subscribePoseInfo(bool required)\n",
        battery_methods + "bool GZBridge::subscribePoseInfo(bool required)\n",
        "bool GZBridge::subscribeBattery(bool required)",
    )

    text = path.read_text()
    soc_update = "\t_battery.setStateOfCharge(math::constrain(static_cast<float>(msg.percentage()) / 100.0f, 0.0f, 1.0f));\n"
    if soc_update not in text:
        current_update = "\t_battery.updateCurrent(math::max(static_cast<float>(msg.current()), 0.0f));\n"
        if text.count(current_update) != 1:
            raise RuntimeError(f"Cannot find battery current update in {path}")
        path.write_text(text.replace(current_update, current_update + soc_update, 1))


def xml_value(value: object) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"Unsupported Gazebo battery value: {value!r}")
    return escape(str(value))


def render_model_plugin(config: dict) -> str:
    required = (
        "battery_name",
        "joint_names",
        "motor_constant",
        "moment_constant",
        "rotor_velocity_slowdown",
        "motor_efficiency",
        "static_power",
        "capacity",
        "initial_charge",
        "open_circuit_voltage_constant_coef",
        "open_circuit_voltage_linear_coef",
        "resistance",
        "smooth_current_tau",
        "reset_charge_after_idle_s",
        "idle_rotor_threshold_rad_s",
    )
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError(f"Missing gazebo_battery_model settings: {', '.join(missing)}")
    if config["battery_name"] != "linear_battery":
        raise ValueError("battery_name must be 'linear_battery' because the PX4 bridge subscribes to that topic")

    lines = [
        '    <plugin filename="MotorPowerSystem"',
        '      name="gz::sim::systems::MotorPowerSystem">',
        f"      <battery_name>{xml_value(config['battery_name'])}</battery_name>",
    ]
    for joint_name in config["joint_names"]:
        lines.append(f"      <joint_name>{xml_value(joint_name)}</joint_name>")
    for key in required[2:]:
        lines.append(f"      <{key}>{xml_value(config[key])}</{key}>")
    lines.append("    </plugin>")
    return "\n".join(lines)


def patch_model(px4_dir: Path, config: dict) -> None:
    model_name = config.get("model", "x500")
    path = px4_dir / f"Tools/simulation/gz/models/{model_name}/model.sdf"
    text = path.read_text()
    plugin = render_model_plugin(config)
    pattern = re.compile(
        r'    <plugin filename="MotorPowerSystem".*?^    </plugin>',
        flags=re.MULTILINE | re.DOTALL,
    )
    if pattern.search(text):
        text = pattern.sub(plugin, text, count=1)
    else:
        anchor = '    <plugin filename="MotorFailurePlugin"'
        if text.count(anchor) != 1:
            raise RuntimeError(f"Cannot find MotorFailurePlugin insertion point in {path}")
        text = text.replace(anchor, plugin + "\n" + anchor, 1)
    path.write_text(text)


def current_revision(px4_dir: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(px4_dir), "rev-parse", "HEAD"], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()

    px4_dir = args.px4_dir.resolve()
    extension_dir = Path(__file__).resolve().parent
    config = yaml.safe_load(args.config.read_text()) or {}
    battery_config = config.get("gazebo_battery_model")
    if not isinstance(battery_config, dict):
        raise ValueError("gazebo_battery_model must be configured in uav_config.yaml")

    revision = current_revision(px4_dir)
    if revision and revision != SUPPORTED_PX4_REVISION:
        print(
            f"WARNING: extension was validated on PX4 {SUPPORTED_PX4_REVISION}, "
            f"but the checkout is {revision}; patch anchors will still be validated."
        )

    install_plugin_sources(px4_dir, extension_dir)
    patch_plugin_build(px4_dir)
    patch_bridge_build(px4_dir)
    patch_bridge_header(px4_dir)
    patch_bridge_source(px4_dir)
    patch_model(px4_dir, battery_config)
    print(f"Battery extension installed in {px4_dir}")


if __name__ == "__main__":
    main()
