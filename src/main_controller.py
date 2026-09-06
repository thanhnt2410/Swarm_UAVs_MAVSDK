import asyncio
import glob
import os
import pickle
import sys
import subprocess
import time
import yaml
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

import pyfiglet
from asyncqt import QEventLoop
# PyQt5
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
# pyrefly: ignore [missing-import]
from PyQt5.QtWidgets import QFileDialog

# ultralytics
from ultralytics import YOLO

# user-defined configuration loader
from config_loader import ConfigLoader
# user-defined interface
from interface_base import logger
from interface_map import DOT_ICON_PATH, Map

# user-defined utils
from utils.drone_utils import (
    clear_mission_logs, export_points_to_gps_log,
    select_mission_plan, uav_rescue_process
)
from utils.qt_utils import get_system_information, draw_table, get_values_from_table
import serial

# user-defined services
from services.drone_service import DroneService
from controllers.stream_controller import StreamController
from mavsdk_server.mavsdk_server_utils import MAVSDKServer
from controllers.telemetry_controller import TelemetryController

# user-defined planning
from planning.path_algorithms import (
    abc_path,
    aco_path,
    astar_path_with_turns,
    find_path_0,
    find_zigzag_path,
    ga_path,
    ga_path_with_turns,
    nn_2opt_path,
    sa_path,
)
from planning.grid import generate_grid, split_grids
from planning.geometry import calculate_new_lat_lon, latlon_to_xy
from planning.uav_analyzer import UAVAnalyzer

# --- Load Application Configuration ---
try:
    config = ConfigLoader()
except (FileNotFoundError, ValueError) as e:
    print(f"[FATAL] Failed to load configuration: {e}")
    sys.exit(1)

RESCUE_UAV_INDEX = config.RESCUE_UAV_INDEX
MISSION_ENERGY_MODEL_DIR = Path(__file__).resolve().parent.parent / "models" / "mission_energy"
MISSION_ENERGY_MODEL_NAME = "linear"
MISSION_ENERGY_FEATURE_COLUMNS = ["total_distance_m", "stop_count"]

# gimbal 
GIMBAL_C12_PATH = os.path.join(os.path.dirname(__file__), "GimbalC12.py")

# cspell: ignore UAVs mavsdk asyncqt figlet ndarray offboard pixmap qgroundcontrol rtcm imwrite dsize fourcc imread
__version__ = "3.20.0"
__current_time__ = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
__current_path__ = os.path.dirname(os.path.abspath(__file__))
__system_info__ = get_system_information()
print("*" * 50 + "\n" + "*" * 50)
print(f"SYSTEM INFO:\n{__system_info__}")
print(f"APP VERSION: {__version__}\nWorking directory: {__current_path__}\n{'*' * 50}")
print(pyfiglet.figlet_format("UAV SWARM CONTROL APP"))
print("*" * 50)
print("CURRENT TIME:", __current_time__)

logger.log(f"Application initializing...", level="info")

class MainController:

    def __init__(self) -> None:
        self.config = config
        self.drone_service = DroneService(config)
        self.stream_controller = StreamController(self)
        self.telemetry_controller = TelemetryController(self)
        self.UAVs = self.drone_service.get_all_uavs() # Keep a reference for legacy UI code
        
        # Initialize UI via Composition
        self.view = Map(config=config)
        self.ui = self.view.ui
        self.logger = logger
        self._mission_energy_models = {}
        self.logger.log(f"Initialize detection model on {self.config.stream['source'].get('device', 'cpu')}...", level="info")
        start_time = time.time()
        for uav_idx in range(1, self.config.MAX_UAV_COUNT + 1):
            if uav_idx in self.UAVs:
                try:
                    from ultralytics import YOLO
                    self.UAVs[uav_idx].detection_model = YOLO(self.config.model_uav_paths[uav_idx])
                except Exception as e:
                    self.logger.log(f"Failed to load YOLO model for UAV {uav_idx}: {e}", level="error")
        self.logger.log(
            f"Detection models loaded successfully in {(time.time() - start_time):3f}s!",
            level="info",
        )
        self.init_application()
        self.logger.log("Application initialized successfully", level="info")

    def _update_action_buttons_state(self, tab_index: int) -> None:
        """Update action buttons based on the selected tab"""
        if tab_index in range(1, self.config.MAX_UAV_COUNT + 1):
            is_on_mission = self.UAVs[tab_index].telemetry.on_mission
            if is_on_mission:
                self._set_pause_button_style("Pause")
            else:
                self._set_pause_button_style("Resume")
        else:
            self._set_pause_button_style("Pause/Resume")

    def _set_pause_button_style(self, text_state: str) -> None:
        """Helper to set text and color for Pause/Resume button"""
        self.ui.btn_pause_resume.setText(text_state)
        if text_state == "Pause":
            # Nền màu Vàng/Cam khi UAV đang bay (Nhấn để Pause)
            self.ui.btn_pause_resume.setStyleSheet(
                "QPushButton{background-color: rgb(252, 175, 62);}\n"
                "QPushButton::pressed{background-color: rgb(255, 0, 0);}"
            )
        elif text_state == "Resume":
            # Nền màu Xanh lá khi UAV đang dừng (Nhấn để Resume)
            self.ui.btn_pause_resume.setStyleSheet(
                "QPushButton{background-color: rgb(138, 226, 52);}\n"
                "QPushButton::pressed{background-color: rgb(255, 0, 0);}"
            )
        else:
            # Nền màu Xanh dương mặc định cho Tab All
            self.ui.btn_pause_resume.setStyleSheet(
                "QPushButton{background-color: rgb(114, 159, 207);}\n"
                "QPushButton::pressed{background-color: rgb(255, 0, 0);}"
            )

    # ---------------------------------------------------------

    def init_application(self) -> None:
        """
        Initialize the application components and configure default settings.
        
        This method sets up the UI, connections, and configuration required for the application
        to run properly. It performs the following tasks:
        1. Sets the default view tabs
        2. Configures button click events
        3. Sets up line edit events
        4. Creates streaming threads
        5. Initializes settings and UI components
        """
        self.logger.log("Initializing application components...", level="info")
        
        # Set default UI views
        self._init_interface_views()
        
        # Setup event handlers
        self._init_event_handlers()
        
        # Create streaming components
        self.stream_controller._create_streaming_threads()
        
        # Configure settings
        self._handling_settings(mode="init")
    
    def _init_interface_views(self) -> None:
        """
        Set up the initial UI views and default tab selections.
        
        This configures the default screens and tabs that are shown when
        the application first starts.
        """
        # Set main view to the first page
        self.ui.stackedWidget.setCurrentIndex(0)
        
        # Set the tab view to the first tab
        self.ui.tabWidget.setCurrentIndex(0)
        
        # Configure initial status indicators
        self._update_status_indicators()
        
        # Mở rộng giới hạn nhập cho các ô kích thước map (mặc định Qt chỉ cho nhập đến 99)
        self.ui.spinBox_5.setMaximum(10000)
        self.ui.spinBox_6.setMaximum(10000)
        self.ui.spinBox_7.setMaximum(10000)
        self.ui.spinBox_8.setMaximum(10000)

    def _init_event_handlers(self) -> None:
        """
        Initialize all event handlers for UI components.
        
        This sets up the connections between UI elements (buttons, line edits)
        and their corresponding handler functions.
        """
        # Connect button click events
        self._button_clicked_event()
        
        # Connect line edit events
        self._line_edit_event()
        
        # Connect custom signals
        self._connect_custom_signals()
        
    def _update_status_indicators(self) -> None:
        """
        Update the status indicators for all UAVs.
        
        This updates the visual indicators showing the connection status,
        battery level, and other status information for each UAV.
        """
        # Update connection status indicators
        for uav_index in range(1, self.config.MAX_UAV_COUNT + 1):
            self.telemetry_controller.set_connection_display(uav_index, self.UAVs[uav_index].telemetry)

    def _connect_custom_signals(self) -> None:
        """
        Connect custom Qt signals to their respective handler functions.
        
        This sets up signal connections for custom events like streaming
        updates and parameter changes.
        """
        # Add any additional signal connections here
        pass

    # ---------------------------<UI Events>---------------------------
    def _button_clicked_event(self) -> None:
        """
        Maps UI button click events to UAV control functions using async tasks.

        Connects buttons to functions for UAV operations like arming, disarming, opening/closing,
        landing, taking off, pausing missions, connecting, returning, and pushing missions.
        Also maps buttons for setting/getting UAV flight info, updating settings, and navigation.

        Buttons mapped:
        - Arm, Disarm, Open/Close, Landing, Take Off, Pause Mission, Connect, Return, Mission, Push Mission
        - Set/Get Flight Info (for each UAV)
        - Update Settings (for 'settings' and 'overview' pages)
        - Go To (for 'settings' and 'overview' pages)
        """
        # Define button mappings for main control functions
        button_mappings = {
            self.ui.btn_arm: lambda: self.uav_arm_callback(self.view.active_tab_index),
            self.ui.btn_disarm: lambda: self.uav_disarm_callback(self.view.active_tab_index),
            self.ui.btn_open_close: lambda: self.uav_toggle_open_callback(self.view.active_tab_index),
            self.ui.btn_landing: lambda: self.uav_land_callback(self.view.active_tab_index),
            self.ui.btn_take_off: lambda: self.uav_takeoff_callback(self.view.active_tab_index),
            self.ui.btn_pause_resume: lambda: self.uav_toggle_pause_mission_callback(self.view.active_tab_index),
            self.ui.btn_connect: lambda: self.uav_connect_callback(self.view.active_tab_index),
            self.ui.btn_rtl: lambda: self.uav_return_callback(self.view.active_tab_index, rtl=True),
            self.ui.btn_return: lambda: self.uav_return_callback(self.view.active_tab_index, rtl=False),
            self.ui.btn_mission: lambda: self.uav_mission_callback(self.view.active_tab_index)
        }
        
        # Connect main control buttons
        for button, callback in button_mappings.items():
            button.clicked.connect(lambda checked=False, cb=callback: asyncio.create_task(cb()))
        
        # Xử lý riêng nút Push Mission để tránh lỗi kẹt Asyncio Loop với QFileDialog
        self.ui.btn_push_mission.clicked.connect(
            lambda: self.uav_push_mission_sync_handler(self.view.active_tab_index)
        )

        # Connect camera toggle button (non-async)
        self.ui.btn_toggle_camera.clicked.connect(
            lambda: self.stream_controller.uav_toggle_camera_callback(self.view.active_tab_index)
        )
        
        # Connect parameter buttons for each UAV
        self._connect_parameter_buttons()
        
        # Connect settings configuration buttons
        self.ui.btn_sett_cf_nSwarms.clicked.connect(
            lambda: self._handling_settings(mode="settings")
        )
        self.ui.btn_ovv_cf_nSwarms.clicked.connect(
            lambda: self._handling_settings(mode="overview")
        )
        
        # Connect GoTo navigation buttons
        self._connect_goto_buttons()
        
        # Connect Simulation button
        self.ui.pushButton.clicked.connect(
            lambda: asyncio.create_task(self.run_simulation_callback())
        )
        
        # Đồng bộ Combobox Map Type với giao diện nhập tham số tương ứng
        self.ui.comboBox_3.currentIndexChanged.connect(self._on_map_type_changed)
        # Chạy 1 lần lúc khởi động để đồng bộ giao diện
        self._on_map_type_changed(self.ui.comboBox_3.currentIndex())
        
    def _connect_parameter_buttons(self) -> None:
        """
        Connect parameter control buttons for each UAV.
        
        This connects the set/get parameter buttons for all UAVs to
        the appropriate handler functions.
        """
        # Connect Set Parameter buttons
        for uav_index in range(1, self.config.MAX_UAV_COUNT + 1):
            idx = uav_index - 1  # Adjust for zero-based indexing
            
            # Create a closure to capture the current UAV index
            def create_set_callback(uav_idx):
                return lambda: asyncio.create_task(self.telemetry_controller.uav_fn_set_flight_info(uav_idx))
            
            def create_get_callback(uav_idx):
                return lambda: asyncio.create_task(self.telemetry_controller.uav_fn_get_flight_info(uav_idx, True))
            
            # Connect Set Parameter button
            self.view.uav_set_param_buttons[idx].clicked.connect(create_set_callback(uav_index))
            
            # Connect Get Parameter button
            self.view.uav_get_param_buttons[idx].clicked.connect(create_get_callback(uav_index))

    def _connect_goto_buttons(self) -> None:
        # GoTo button mapping for settings and overview pages
        for uav_index in range(self.config.MAX_UAV_COUNT + 1):  # 0-6 for all UAVs plus all-UAV control
            # Create closures to capture the current UAV index
            def create_goto_settings_callback(uav_idx):
                return lambda: asyncio.create_task(
                    self.uav_goto_callback(uav_index=uav_idx, page="settings")
                )
            
            def create_goto_overview_callback(uav_idx):
                return lambda: asyncio.create_task(
                    self.uav_goto_callback(uav_index=uav_idx, page="overview")
                )
            
            # Connect Settings page GoTo button
            self.view.uav_sett_goTo_buttons[uav_index].clicked.connect(
                create_goto_settings_callback(uav_index)
            )
            
            # Connect Overview page GoTo button
            self.view.uav_ovv_goTo_buttons[uav_index].clicked.connect(
                create_goto_overview_callback(uav_index)
            )

    def _line_edit_event(self) -> None:
        """
        Connect line edit events to their handler functions.
        
        This connects the returnPressed event of command input fields
        to the process_command function for each UAV.
        """
        for index in range(self.config.MAX_UAV_COUNT):
            # Create a closure to capture the current UAV index
            def create_command_callback(uav_idx):
                return lambda: asyncio.create_task(self.process_command(uav_idx))
            
            # Connect the returnPressed event to the process_command function
            self.view.uav_update_commands[index].returnPressed.connect(
                create_command_callback(index + 1)
            )

            
        
        
    async def _connect_stream_signal(self, uav_index):
        """Connect the streaming thread signal to the display slot"""
        try:
            # Try to disconnect any existing connection to prevent duplicates
            self.UAVs[uav_index].stream_thread.change_image_signal.disconnect()
        except Exception:
            # Ignore errors if there was no existing connection
            pass
            
        # Connect the signal to the slot with queued connection type
        self.UAVs[uav_index].stream_thread.change_image_signal.connect(
            self.stream_on_uav_screen,
            Qt.QueuedConnection
        )
    # //-/////////////////////////////////////////////////////////////

    def _handling_settings(self, mode="init") -> None:
        """
        Handle configuration settings for the interface.
        
        This method manages configuration settings across different modes:
        - init: Load initial settings from configuration
        - settings: Apply settings from the Settings tab
        - overview: Apply settings from the Overview tab
        
        Args:
            mode (str): The mode to handle ('init', 'settings', or 'overview')
        """
        try:
            self.logger.log(f"Handling settings in '{mode}' mode", level="info")
            
            # Handle checkbox states and related UAV settings
            self._handling_checkboxes(mode=mode)
            
            # Handle table data and connection settings
            self._handling_tables(mode=mode)
            
        except Exception as e:
            self.logger.log(f"Error handling settings in '{mode}' mode: {e}", level="error")
            self.view.popup_msg(
                msg=f"Error handling settings: {e}", 
                src_msg="_handling_settings", 
                type_msg="Error"
            )

    def _handling_checkboxes(self, mode="init") -> None:
        """
        Handle checkbox states and update UAV detection/streaming settings.
        
        This method synchronizes checkbox states between UI elements and UAV settings
        based on the specified mode.
        
        Args:
            mode (str): The mode to handle ('init', 'settings', or 'overview')
            
        Returns:
            None
        """
        
        try:
            if mode == "init":
                # Initialize UI checkboxes based on configuration
                for i, widget in enumerate(self.view.sett_checkBox_detect_lists):
                    widget.setChecked(self.UAVs[i + 1].config.detection_enabled)
                    
                for i, widget in enumerate(self.view.ovv_checkBox_detect_lists):
                    widget.setChecked(self.UAVs[i + 1].config.detection_enabled)
                    
                for i, widget in enumerate(self.view.sett_checkBox_active_lists):
                    widget.setChecked(self.UAVs[i + 1].config.streaming_enable)
                    
            elif mode == "settings":
                # Update UAV settings from Settings page UI
                for i, widget in enumerate(self.view.sett_checkBox_detect_lists):
                    self.UAVs[i + 1].config.detection_enabled = widget.isChecked()
                    
                # Sync to Overview page
                for i, widget in enumerate(self.view.ovv_checkBox_detect_lists):
                    widget.setChecked(self.UAVs[i + 1].config.detection_enabled)
                    
                # Update streaming settings
                for i, widget in enumerate(self.view.sett_checkBox_active_lists):
                    self.UAVs[i + 1].config.streaming_enable = widget.isChecked()
                    
            elif mode == "overview":
                # Update UAV settings from Overview page UI
                for i, widget in enumerate(self.view.ovv_checkBox_detect_lists):
                    self.UAVs[i + 1].config.detection_enabled = widget.isChecked()
                    
                # Sync to Settings page
                for i, widget in enumerate(self.view.sett_checkBox_detect_lists):
                    widget.setChecked(self.UAVs[i + 1].config.detection_enabled)
                    
            self.logger.log(f"Checkbox settings updated in '{mode}' mode", level="debug")
            
        except Exception as e:
            self.logger.log(f"Error handling checkboxes in '{mode}' mode: {e}", level="error")
            self.view.popup_msg(
                msg=f"Error handling checkboxes: {e}", 
                src_msg="_handling_checkboxes", 
                type_msg="Error"
            )

    def _handling_tables(self, mode="init") -> None:
        """
        Update table data and related UAV connection settings.
        
        This method handles table data for UAV connection and streaming configuration
        according to the specified mode.
        
        Args:
            mode (str): The mode to handle ('init', 'settings', or 'overview')
            
        Returns:
            None
        """
        
        try:
            # Common setup for all modes
            headers = ["id", "connection_address", "streaming_address"]
            connection_allow_indexes = self._get_enabled_uav_indexes("connection")
            streaming_enabled_indexes = self._get_enabled_uav_indexes("streaming")
            
            if mode == "init":
                # Prepare initial table data from configuration
                data = {
                    headers[0]: [uav_index for uav_index in range(1, self.config.MAX_UAV_COUNT + 1)],
                    headers[1]: [
                        f"{self.UAVs[uav_index].config.system_address} -p {self.UAVs[uav_index].system._port}"
                        for uav_index in range(1, self.config.MAX_UAV_COUNT + 1)
                    ],
                    headers[2]: [
                        self.UAVs[uav_index].config.streaming_address
                        for uav_index in range(1, self.config.MAX_UAV_COUNT + 1)
                    ],
                }
                nSwarms = len(connection_allow_indexes)
                
            else:
                # Get number of swarms from appropriate UI element
                if mode == "settings":
                    nSwarms = min(
                        int(self.ui.nSwarms_sett.value()), 
                        len(connection_allow_indexes)
                    )
                    data = get_values_from_table(self.ui.table_uav_large, headers=headers)
                else:  # overview mode
                    nSwarms = min(
                        int(self.ui.nSwarms_ovv.value()), 
                        len(connection_allow_indexes)
                    )
                    data = get_values_from_table(self.ui.table_uav_small, headers=headers)
                
                # Update UAV configuration from table data
                self._update_uav_config_from_table(data, connection_allow_indexes)
            
            # Update tables with current data
            self._update_tables(
                data=data,
                connection_allow_indexes=connection_allow_indexes,
                streaming_enabled_indexes=streaming_enabled_indexes,
                nSwarms=nSwarms,
                headers=headers,
            )
            
            self.logger.log(f"Table settings updated in '{mode}' mode", level="debug")
            
        except Exception as e:
            self.logger.log(f"Error handling tables in '{mode}' mode: {e}", level="error")
            self.view.popup_msg(
                msg=f"Error handling tables: {e}", 
                src_msg="_handling_tables", 
                type_msg="Error"
            )

    def _get_enabled_uav_indexes(self, feature_type):
        """
        Get indexes of UAVs with a specific feature enabled.
        
        Args:
            feature_type (str): The feature to check ('connection' or 'streaming')
            
        Returns:
            list: List of UAV indexes with the specified feature enabled
        """
        
        if feature_type == "connection":
            return [
                index + 1 for index in range(self.config.MAX_UAV_COUNT) 
                if self.UAVs[index + 1].config.connection_allow
            ]
        elif feature_type == "streaming":
            return [
                index + 1 for index in range(self.config.MAX_UAV_COUNT) 
                if self.UAVs[index + 1].config.streaming_enable
            ]
        else:
            return []

    def _update_uav_config_from_table(self, data, connection_allow_indexes):
        """
        Update UAV configuration from table data.
        
        Args:
            data (dict): Table data containing connection and streaming addresses
            connection_allow_indexes (list): Indexes of UAVs with connection allowed
            
        Returns:
            None
        """
        
        # Process each UAV's settings
        for index in range(self.config.MAX_UAV_COUNT):
            uav_index = index + 1
            if uav_index in connection_allow_indexes:
                # Parse connection address into components
                conn_address = data["connection_address"][index]
                address_parts, client_port = conn_address.split("-p")
                proto, server_parts = address_parts.split(":", 1)
                server_host = server_parts.split(":", 1)[0].replace("//", "")
                bind_port = server_parts.split(":", 1)[1] if ":" in server_parts else "0"
                
                # Update MAVSDK server configuration
                self.UAVs[uav_index].server["shell"] = MAVSDKServer(
                    id=uav_index,
                    protocol=proto,
                    server_host=server_host,
                    port=int(client_port),
                    bind_port=int(bind_port),
                )
                
                # Update connection addresses
                self.UAVs[uav_index].config.system_address = f"{proto}:{server_parts}"
                self.UAVs[uav_index].system._port = int(client_port)
                
                # Update streaming address
                self.UAVs[uav_index].config.streaming_address = data["streaming_address"][index].strip()
        
        # Reset connection and streaming status after configuration change
        for uav_index in range(1, self.config.MAX_UAV_COUNT + 1):
            self.UAVs[uav_index].telemetry.connected = False
            self.UAVs[uav_index].telemetry.streaming_status = False
        
        # Recreate streaming threads with new configuration
        self.stream_controller._create_streaming_threads()
        self.logger.log("Updated UAV configuration from table data", level="info")

    def _update_tables(
        self, data, connection_allow_indexes, streaming_enabled_indexes, nSwarms, headers
    ) -> None:
        """
        Update UAV tables with current configuration data.
        
        Args:
            data (dict): Table data to display
            connection_allow_indexes (list): Indexes of UAVs with connection allowed
            streaming_enabled_indexes (list): Indexes of UAVs with streaming enabled
            nSwarms (int): Number of swarm UAVs to display
            headers (list): Column headers for the table
            
        Returns:
            None
        """
        # Convert to DataFrame if needed
        df = pd.DataFrame.from_dict(data) if not isinstance(data, pd.DataFrame) else data
        
        # Update large table (settings page)
        draw_table(
            table_widget=self.ui.table_uav_large,
            data=df,
            connection_allow_indexes=connection_allow_indexes[:nSwarms],
            streaming_enabled_indexes=streaming_enabled_indexes,
            headers=headers,
        )
        
        # Update small table (overview page)
        draw_table(
            table_widget=self.ui.table_uav_small,
            data=df,
            connection_allow_indexes=connection_allow_indexes[:nSwarms],
            streaming_enabled_indexes=streaming_enabled_indexes,
            headers=headers,
        )
        
        # Update nSwarms value in both settings and overview pages
        self.ui.nSwarms_sett.setValue(nSwarms)
        self.ui.nSwarms_ovv.setValue(nSwarms)
        
        self.logger.log(f"Updated UAV tables with {nSwarms} swarms", level="debug")

    # ////////////////////////////////////////////////////////////////

    async def process_command(self, uav_index) -> None:
        """
        Processes a command for a specific UAV based on the given index.
        Args:
            uav_index (int): The index of the UAV to process the command for.
        Returns:
            None
        Raises:
            Exception: If an error occurs during command processing.
        The function performs the following steps:
        1. Checks if the UAV is connected and allowed to receive commands.
        2. Retrieves the command text from the corresponding UAV update command input.
        3. If the command is "hold", it instructs the UAV to hold its position.
        4. If the command is a movement or gimbal control command, it parses the command and value,
           and performs the corresponding action:
           - Movement commands: "forward", "backward", "left", "right", "up", "down"
           - Gimbal control commands: "pitch", "yaw", "gimbal"
        5. Clears the input after processing the command.
        6. Displays an error message if an invalid input is encountered.
        """

        try:
            text = self.view.uav_update_commands[uav_index - 1].text()
            if "=" not in text:
                command = text.strip().lower()

                if command == "gimbal":
                    self.open_gimbal_c12()
                else:
                    self.view.popup_msg(
                        f"Unknown command: {command}",
                        src_msg="process_command",
                        type_msg="Error",
                    )

                self.view.uav_update_commands[uav_index - 1].clear()
                return
            
            if not (
                self.UAVs[uav_index].telemetry.connected
                and self.UAVs[uav_index].config.connection_allow
            ):
                return

            text = self.view.uav_update_commands[uav_index - 1].text()

            if text.lower().strip() == "hold":
                await self.UAVs[uav_index].system.action.hold()
            else:
                command, value = str(text).split("=")
                command = command.strip().lower()
                value = value.strip().lower()

                # NOTE: if command <do something more here>

                # * 1. control movement command
                if command in ["forward", "backward", "left", "right", "up", "down"]:
                    distance = float(value)
                    await self.drone_service.uav_fn_goto_distance(
                        uav_index=uav_index,
                        distance=distance,
                        direction=command,
                    )

                # * 2. control gimbal command
                if command in ["pitch", "yaw"]:
                    angle = float(value)
                    control_value = (
                        {"pitch": angle, "yaw": 0}
                        if command == "pitch"
                        else {"pitch": 0, "yaw": angle}
                    )
                    await self.drone_service.uav_fn_control_gimbal(
                        uav_index=uav_index, control_value=control_value
                    )

        except Exception as e:
            self.view.popup_msg(
                f"Invalid input: {repr(e)}", src_msg="process_command", type_msg="Error"
            )
    # open gimbalc12...................................................................
    def open_gimbal_c12(self):
        """Mở cửa sổ điều khiển Gimbal C12 (file GimbalC12.py cùng thư mục)."""
        try:
            subprocess.Popen([sys.executable, GIMBAL_C12_PATH])
        except Exception as e:
            self.view.popup_msg(
                f"Không mở được GimbalC12.py: {repr(e)}",
                src_msg="open_gimbal_c12",
                type_msg="Error",
            )
    # -----------------------< UAV buttons callback functions >-----------------------
    async def uav_connect_callback(self, uav_index) -> None:
        """
        Asynchronous callback function to handle UAV connection. Connect to a specific UAV or all UAVs.
        
        It performs several steps:
        1. Initializes the server for the UAV.
        2. Connects to the UAV system and Checks the connection status.
        3. Updates the connection status display.
        4. Overwrites and exports UAV parameters.
        5. Continuously updates the UAV status.
        If the UAV index is not within the valid range, it attempts to connect to all UAVs.
        
        Args:
            uav_index (int): The UAV to connect to (1-MAX_UAV_COUNT), or 0 for all UAVs
            
        Returns:
            None
            
        Raises:
            Exception: If there is an error during the connection process, it logs the error and displays a popup message.
        """

        # Handle the case of connecting to all UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            connect_tasks = [
                self.uav_connect_callback(i) for i in range(1, self.config.MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*connect_tasks)
            return

        # Skip if connection is not allowed
        if not self.UAVs[uav_index].config.connection_allow:
            self.view.update_terminal(f"[INFO] Connection not allowed for UAV {uav_index}")
            return
            
        try:
            self.view.update_terminal(f"[INFO] Sent CONNECT command to UAV {uav_index}")
            uav_data = self.drone_service.get_uav(uav_index)
            if uav_data:
                uav_data.telemetry.connected = False
                self.telemetry_controller.set_connection_display(uav_index, uav_data.telemetry)

            await self.drone_service.connect(uav_index)

            self.view.update_terminal(f"[INFO] Received CONNECT signal from UAV {uav_index}")
            self.telemetry_controller.set_connection_display(uav_index, self.UAVs[uav_index].telemetry)

            await self.telemetry_controller.uav_fn_get_status(uav_index, verbose=True)
            # The map is now updated automatically by uav_fn_get_position.

        except Exception as e:
            uav_data = self.drone_service.get_uav(uav_index)
            if uav_data:
                uav_data.telemetry.connected = False
                self.telemetry_controller.set_connection_display(uav_index, uav_data.telemetry)
            self.logger.log(f"Connection error to UAV {uav_index}: {repr(e)}", level="error")
            self.view.popup_msg( # type: ignore
                f"Connection error to UAV {uav_index}: {repr(e)}",
                src_msg="uav_connect_callback",
                type_msg="error",
            )

    async def uav_arm_callback(self, uav_index) -> None:
        """
        Arm a specific UAV or all UAVs.
        
        This method sends an ARM command to the specified UAV(s), waits for
        confirmation, and updates the arming status in the UI.
        
        Args:
            uav_index (int): The UAV to arm (1-MAX_UAV_COUNT), or 0 for all available UAVs
            
        Returns:
            None
        """
        
        avail_uav_indexes = [uav['id'] for uav in self.config.uav['uavs'] if uav['id'] != self.config.RESCUE_UAV_INDEX]
        # Handle the case of arming all available UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            arm_tasks = [
                self.uav_arm_callback(i) for i in avail_uav_indexes
            ]
            await asyncio.gather(*arm_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            self.view.update_terminal(f"[INFO] Sent ARM command to UAV {uav_index}")
            await self.drone_service.arm(uav_index)
            await asyncio.sleep(3)
            await self.uav_disarm_callback(uav_index)
            self.telemetry_controller._update_uav_info_display(uav_index)
            
        except Exception as e:
            self.drone_service.get_uav(uav_index).telemetry.armed = "DISARMED"
            self.telemetry_controller._update_uav_info_display(uav_index)
            
            # Lấy thông báo lỗi chi tiết thay vì dùng repr(e)
            error_detail = str(e)
            if hasattr(e, '_result'):
                error_detail = f"{e._result.result_str} (Code: {e._result.result})"
                
            self.logger.log(f"Arming error: {error_detail}", level="error")
            self.view.popup_msg(f"Error: {error_detail}", src_msg="uav_arm_callback", type_msg="Error")

    async def uav_disarm_callback(self, uav_index) -> None:
        """
        Disarm a specific UAV or all UAVs.
        
        This method sends a DISARM command to the specified UAV(s) and updates
        the arming status in the UI.
        
        Args:
            uav_index (int): The UAV to disarm (1-MAX_UAV_COUNT), or 0 for all UAVs
            
        Returns:
            None
        """
        
        # Handle the case of disarming all UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            disarm_tasks = [
                self.uav_disarm_callback(i) for i in range(1, self.config.MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*disarm_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            self.view.update_terminal(f"[INFO] Sent DISARM command to UAV {uav_index}")
            await self.drone_service.disarm(uav_index)
            self.telemetry_controller._update_uav_info_display(uav_index)
            
        except Exception as e:
            self.logger.log(f"Disarming error: {repr(e)}", level="error")
            self.view.popup_msg(
                f"Error: {repr(e)}", src_msg="uav_disarm_callback", type_msg="Error"
            )

    async def uav_takeoff_callback(self, uav_index) -> None:
        """
        Initiate takeoff for a specific UAV or all UAVs.
        
        This method sends a TAKEOFF command to the specified UAV(s), arms the UAV,
        initiates takeoff, and updates the status in the UI.
        
        Args:
            uav_index (int): The UAV to command takeoff (1-MAX_UAV_COUNT), or 0 for all available UAVs
            
        Returns:
            None
        """
        
        avail_uav_indexes = [uav['id'] for uav in self.config.uav['uavs'] if uav['id'] != self.config.RESCUE_UAV_INDEX]
        # Handle the case of taking off all available UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            takeoff_tasks = [
                self.uav_takeoff_callback(i) for i in avail_uav_indexes
            ]
            await asyncio.gather(*takeoff_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            self.view.update_terminal(f"[INFO] Sent TAKEOFF command to UAV {uav_index}")
            await self.drone_service.takeoff(uav_index)
            await self._save_initial_position(uav_index)
            self.telemetry_controller._update_uav_info_display(uav_index)
            
        except Exception as e:
            self.logger.log(f"Takeoff error: {repr(e)}", level="error")
            self.view.popup_msg(
                f"Error: {repr(e)}", src_msg="uav_takeoff_callback", type_msg="Error"
            )

    async def _save_initial_position(self, uav_index):
        """Save the initial position of a UAV to a file."""
        # Update initial position from current position
        self.UAVs[uav_index].config.init_params["latitude"] = round(
            self.UAVs[uav_index].telemetry.latitude, 12
        )
        self.UAVs[uav_index].config.init_params["longitude"] = round(
            self.UAVs[uav_index].telemetry.longitude, 12
        )
        
        # Save to YAML file
        yaml_file = self.config.config_dir / 'init_pos_uavs.yaml'
        try:
            data = {}
            if os.path.exists(yaml_file):
                with open(yaml_file, "r") as f:
                    data = yaml.safe_load(f) or {}
            
            uav_key = f"uav_{uav_index}"
            if uav_key not in data:
                data[uav_key] = {}
                
            data[uav_key]["latitude"] = self.UAVs[uav_index].config.init_params["latitude"]
            data[uav_key]["longitude"] = self.UAVs[uav_index].config.init_params["longitude"]
            
            with open(yaml_file, "w") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            self.logger.log(f"Failed to save initial position to YAML: {e}", level="error")

    async def uav_land_callback(self, uav_index) -> None:
        """
        Command a specific UAV or all UAVs to land.
        
        This method sends a LANDING command to the specified UAV(s) and updates
        the mode status in the UI.
        
        Args:
            uav_index (int): The UAV to command landing (1-MAX_UAV_COUNT), or 0 for all UAVs
            
        Returns:
            None
        """
        
        # Handle the case of landing all UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            landing_tasks = [
                self.uav_land_callback(i) for i in range(1, self.config.MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*landing_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            self.view.update_terminal(f"[INFO] Sent LANDING command to UAV {uav_index}")
            await self.drone_service.land(uav_index)
            self.telemetry_controller._update_uav_info_display(uav_index)
            
        except Exception as e:
            self.logger.log(f"Landing error: {repr(e)}", level="error")
            self.view.popup_msg(f"Error: {repr(e)}", src_msg="uav_land_callback", type_msg="Error")

    async def uav_return_callback(self, uav_index, rtl=False) -> None:
        """
        Command a specific UAV or all UAVs to return.
        
        This method sends either a Return-To-Launch (RTL) command or a return to
        initial position command to the specified UAV(s) and updates the status in the UI.
        
        Args:
            uav_index (int): The UAV to command return (1-MAX_UAV_COUNT), or 0 for all available UAVs
            rtl (bool): If True, use RTL mode (land at return point), otherwise just return to position
            
        Returns:
            None
        """
        
        # Handle the case of returning all available UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            return_tasks = [
                self.uav_return_callback(i, rtl=rtl) for i in range(1, self.config.MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*return_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            # Get position information
            init_latitude = self.UAVs[uav_index].config.init_params["latitude"]
            init_longitude = self.UAVs[uav_index].config.init_params["longitude"]
            current_latitude = self.UAVs[uav_index].telemetry.latitude
            current_longitude = self.UAVs[uav_index].telemetry.longitude
            
            if rtl:
                # Return to launch (return and land)
                self.view.update_terminal(f"[INFO] Sent RTL command to UAV {uav_index}")
                
                # If already at initial position, just land
                if (init_latitude, init_longitude) == (current_latitude, current_longitude):
                    self.view.update_terminal(
                        f"[INFO] UAV {uav_index} is already at the initial position, landing..."
                    )
                    await self.drone_service.land(uav_index)
                else:
                    await self.drone_service.return_to_launch(uav_index)
            else:
                # Return to initial position without landing
                self.view.update_terminal(
                    f"[INFO] Sent RETURN command to UAV {uav_index} to lat: {init_latitude} long: {init_longitude}"
                )
                await self.drone_service.goto_location(uav_index, init_latitude, init_longitude)
                self.UAVs[uav_index].telemetry.flight_mode = "RETURN" # goto_location sets a different mode
            
            # Update display
            self.telemetry_controller._update_uav_info_display(uav_index)
            
            # Clean up mission logs
            clear_mission_logs(uav_index, save_dir=__current_path__)
            
        except Exception as e:
            self.logger.log(f"Return error: {repr(e)}", level="error")
            self.view.popup_msg(
                f"Error: {repr(e)}", src_msg="uav_return_callback", type_msg="Error"
            )

    async def uav_mission_callback(self, uav_index) -> None:
        """NOTE: convert file points to .plan file as in ./data/mission.plan
        Executes a mission for a specified UAV or all UAVs if uav_index is 0.

        Args:
            uav_index (int): The index of the UAV to execute the mission for. If 0, the mission is executed for all UAVs.

        Returns:
            None

        Raises:
            Exception: If there is an error during the mission execution.

        The function performs the following steps:
        1. Checks if the UAV is connected.
        2. Reads mission points from a file and creates mission items.
        3. Uploads the mission to the UAV.
        4. Arms the UAV and starts the mission.
        5. Monitors mission progress and initiates return to launch upon mission completion.
        6. Updates the UAV's mode status and information view.
        7. If uav_index is 0, executes the mission for all UAVs concurrently.
        """
        
        avail_uav_indexes = [uav['id'] for uav in self.config.uav['uavs'] if uav['id'] != self.config.RESCUE_UAV_INDEX]
        # Handle the case of missions for all UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            mission_tasks = [
                self.uav_mission_callback(i) for i in range(1, self.config.MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*mission_tasks)
            return
        
        # Handle regular UAV mission
        if uav_index in avail_uav_indexes:
            await asyncio.gather(
                self._execute_standard_mission(uav_index),
            )
        
        # Handle rescue UAV mission
        elif uav_index == self.config.RESCUE_UAV_INDEX:
            
            await self.uav_fn_rescue()
            
    async def _execute_standard_mission(self, uav_index, plan_file=None):
        """Execute a standard mission for a regular UAV."""
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            # Health check before mission
            self.view.update_terminal(
                "Waiting for drone to have a global position estimate...", uav_index=uav_index
            )
            self.logger.log(f"UAV-{uav_index} -- Global position for estimate OK", level="info")
            
            # Clear detection log files
            detection_log_files = glob.glob(f"{__current_path__}/logs/rescue_pos/*.log")
            for f in detection_log_files:
                os.remove(f)
            
            # Start new mission
            self.view.update_terminal(f"[INFO] Sent MISSION command to UAV {uav_index}")
            self.UAVs[uav_index].telemetry.on_mission = True
            self.UAVs[uav_index].telemetry.mission_start_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            if self.view.active_tab_index == uav_index:
                self._set_pause_button_style("Pause")
            
            if plan_file is None:
                plan_file = f"{__current_path__}/logs/points/reduced_point{uav_index}.plan"
                
            self.view.update_terminal(f"[INFO] Bắt đầu nạp {plan_file} và tiến hành cất cánh...", uav_index=0)
            progress_task = asyncio.create_task(self.monitor_mission_progress(uav_index))
            position_task = asyncio.create_task(self._track_uav_position_on_map(uav_index))

            await self.drone_service.do_mission_and_wait(uav_index, plan_file)
            
            background_tasks = [progress_task, position_task]
            for task in background_tasks:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*background_tasks, return_exceptions=True)
            # Update display
            self.telemetry_controller._update_uav_info_display(uav_index)
            
            # Check if mission is finished and initiate return if needed
            if await self.drone_service.get_uav(uav_index).system.mission.is_mission_finished():
                clear_mission_logs(uav_index, save_dir=__current_path__)
                
            self.UAVs[uav_index].telemetry.on_mission = False
            if self.view.active_tab_index == uav_index:
                self._set_pause_button_style("Resume")
                
        except Exception as e:
            self.logger.log(f"Mission error: {repr(e)}", level="error")
            self.view.popup_msg(
                f"Error: {repr(e)}", src_msg="uav_mission_callback", type_msg="Error"
            )
            
    def uav_push_mission_sync_handler(self, uav_index) -> None:
        """
        Synchronous handler for Push Mission to prevent QFileDialog from blocking the asyncio event loop.
        """
        
        print(f"\n[DEBUG 1] Button Push Mission clicked! uav_index={uav_index}")

        avail_uav_indexes = [uav['id'] for uav in self.config.uav['uavs'] if uav['id'] != self.config.RESCUE_UAV_INDEX]
        # Handle pushing mission to ALL UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            mission_file = QFileDialog.getOpenFileName(
                parent=self,
                caption="Select Mission File for ALL UAVs",
                directory=str(__current_path__),
                filter="Mission Files (*.plan *.txt *.TXT);;All Files (*)",
            )[0]
            
            if not mission_file:
                return
                
            push_tasks = []
            for i in avail_uav_indexes:
                if self._check_uav_connection(i):
                    self.view.update_terminal(f"[INFO] Sent PUSH MISSION command to UAV {i}")
                    push_tasks.append(
                        self.drone_service.uav_fn_upload_mission(uav_index=i, mission_plan_file=mission_file)
                    )
                    self.UAVs[i].telemetry.flight_mode = "Mission uploaded"
                    self.telemetry_controller._update_uav_info_display(i)

            if push_tasks:
                asyncio.create_task(asyncio.gather(*push_tasks))
            else:
                self.view.popup_msg("No connected UAVs to push mission to.", src_msg="Push Mission", type_msg="Warning")
            return
            
        # Handle pushing mission to SINGLE UAV
        if not self._check_uav_connection(uav_index):
            self.view.popup_msg(f"Please connect UAV {uav_index} first!", src_msg="Push Mission", type_msg="Warning")
            return

        plans_log_dir = self.config.SRC_DIR / "logs" / "points"
        if not os.path.exists(plans_log_dir):
            os.makedirs(plans_log_dir, exist_ok=True)

        # Safely open file dialog outside of async context
        mission_file = QFileDialog.getOpenFileName(
            parent=self,
            caption="Select Mission File",
            directory=str(__current_path__),
            filter="Mission Files (*.plan *.txt *.TXT);;All Files (*)",
        )[0]
        
        if not mission_file:
            return
            
        # Spawn async task for MAVSDK upload
        asyncio.create_task(self._async_push_mission(uav_index, mission_file))

    async def _async_push_mission(self, uav_index, mission_file):
        try:
            self.view.update_terminal(f"[INFO] Uploading mission from {mission_file} to UAV {uav_index}")
            await self.drone_service.uav_fn_upload_mission(uav_index=uav_index, mission_plan_file=mission_file)
            
            # 2. Ép con trỏ waypoint bắt đầu từ điểm xuất phát (index 0)
            await self.UAVs[uav_index].system.mission.set_current_mission_item(0)

            # Update status
            self.UAVs[uav_index].telemetry.flight_mode = "Mission uploaded"
            self.telemetry_controller._update_uav_info_display(uav_index)
            
        except Exception as e:
            logger.log(f"Mission push error: {repr(e)}", level="error")
            self.view.popup_msg(f"Error pushing mission: {repr(e)}", src_msg="Push Mission", type_msg="Error")

    async def uav_toggle_pause_mission_callback(self, uav_index) -> None:
        """
        Toggle pause/resume mission for a specific UAV or all UAVs.
        
        This method pauses an ongoing mission or resumes a paused mission
        for the specified UAV(s).
        
        Args:
            uav_index (int): The UAV to toggle mission state (1-MAX_UAV_COUNT),
                            or 0 for all UAVs
            
        Returns:
            None
        """

        # Handle the case of toggling all UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            toggle_tasks = [
                self.uav_toggle_pause_mission_callback(i) for i in range(1, self.config.MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*toggle_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            # Determine current mission state and toggle it
            is_on_mission = self.UAVs[uav_index].telemetry.on_mission
            
            if is_on_mission:
                # Pause the mission
                self.view.update_terminal(f"[INFO] Sent PAUSE MISSION command to UAV {uav_index}")
                await self.drone_service.pause_mission(uav_index)
                if self.view.active_tab_index == uav_index:
                    self._set_pause_button_style("Resume")
            else:
                # Resume the mission
                self.view.update_terminal(f"[INFO] Sent RESUME MISSION command to UAV {uav_index}")
                await self.drone_service.start_mission(uav_index)
                if self.view.active_tab_index == uav_index:
                    self._set_pause_button_style("Pause")
            
            # Update display
            self.telemetry_controller._update_uav_info_display(uav_index)
            
        except Exception as e:
            self.logger.log(f"Mission toggle error: {repr(e)}", level="error")
            self.view.popup_msg(
                f"Error toggling mission: {repr(e)}",
                src_msg="uav_toggle_pause_mission_callback",
                type_msg="Error"
            )

    async def uav_toggle_open_callback(self, uav_index) -> None:
        """
        Toggle actuator open/close state for a specific UAV or all UAVs.
        
        This method toggles the state of the actuator (e.g., payload, gripper)
        for the specified UAV(s) by controlling the gimbal pitch.
        
        Args:
            uav_index (int): The UAV to toggle actuator (1-MAX_UAV_COUNT),
                            or 0 for all UAVs
            
        Returns:
            None
        """
        
        # Handle the case of toggling all UAVs
        if uav_index not in range(1, self.config.MAX_UAV_COUNT + 1):
            toggle_tasks = [
                self.uav_toggle_open_callback(i) for i in range(1, self.config.MAX_UAV_COUNT + 1)
                if self.UAVs[i].config.connection_allow
            ]
            await asyncio.gather(*toggle_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:            
            self.view.update_terminal(f"[INFO] Toggling actuator for UAV {uav_index}")
            await self.drone_service.toggle_actuator(uav_index)
            self.telemetry_controller._update_uav_info_display(uav_index)
            
        except Exception as e:
            self.logger.log(f"Actuator toggle error: {repr(e)}", level="error")
            self.view.popup_msg(
                f"Error toggling actuator: {repr(e)}",
                src_msg="uav_toggle_open_callback",
                type_msg="Error"
            )


    async def uav_goto_callback(self, uav_index, page="settings", *args) -> None:
        
        try:
            # Get coordinates from the appropriate page
            longitude, latitude = self._get_coordinates_from_page(page, uav_index)
            # Ensure coordinates are valid
            if longitude is None or latitude is None:
                self.logger.log("Invalid coordinates for goto command", level="warning")
                self.view.popup_msg(
                    "Invalid coordinates for goto command",
                    src_msg="uav_goto_callback",
                    type_msg="Warning"
                )
                return
                
            # Sync coordinates between settings and overview pages
            self._sync_coordinates_between_pages(longitude, latitude)
            
            # Execute goto command for specific UAV or all UAVs
            if uav_index in range(1, self.config.MAX_UAV_COUNT + 1):
                # Skip if UAV is not connected or not allowed
                if not self._check_uav_connection(uav_index):
                    return
                # Send the command
                self.view.update_terminal(
                    f"[INFO] Sent GOTO command to UAV {uav_index}: lat={latitude}, lon={longitude}")
                await self.drone_service.goto_location(uav_index, latitude, longitude)
                self.telemetry_controller._update_uav_info_display(uav_index)
            else:
                # Command all UAVs to go to the same position
                goto_tasks = []
                for i in range(1, self.config.MAX_UAV_COUNT + 1):
                    if self._check_uav_connection(i):
                        self.view.update_terminal(
                            f"[INFO] Sent GOTO command to UAV {i}: lat={latitude}, lon={longitude}"
                        )
                        goto_tasks.append(self.drone_service.goto_location(i, latitude, longitude))
                        self.telemetry_controller._update_uav_info_display(i)
                        
                if goto_tasks:
                    await asyncio.gather(*goto_tasks)
                
        except Exception as e:
            self.logger.log(f"Goto error: {repr(e)}", level="error")
            self.view.popup_msg(
                f"Error in goto command: {repr(e)}",
                src_msg="uav_goto_callback",
                type_msg="Error"
            )

    def _get_coordinates_from_page(self, page, uav_index):
        """Get coordinates from the specified page with fallback to defaults."""
        
        # Set default coordinates (offset by UAV index to avoid collisions)
        default_longitude = self.UAVs[uav_index].config.init_params["longitude"] if uav_index > 0 else self.config.init_pos['uav_1']['longitude']
        default_latitude = self.UAVs[uav_index].config.init_params["latitude"] if uav_index > 0 else self.config.init_pos['uav_1']['latitude']
        
        # Get coordinates from the specified page
        if page == "settings":
            longitude_text = self.ui.lineEdit_sett_longitude.text().strip()
            latitude_text = self.ui.lineEdit_sett_latitude.text().strip()
        else:  # overview page
            longitude_text = self.ui.lineEdit_ovv_longitude.text().strip()
            latitude_text = self.ui.lineEdit_ovv_latitude.text().strip()
        
        # Parse coordinates with fallback to defaults
        try:
            longitude = float(longitude_text) if longitude_text else default_longitude
            latitude = float(latitude_text) if latitude_text else default_latitude
            return longitude, latitude
        except ValueError:
            self.logger.log(f"Invalid coordinate format: lon={longitude_text}, lat={latitude_text}", level="error")
            return None, None

    def _sync_coordinates_between_pages(self, longitude, latitude):
        """Synchronize coordinates between settings and overview pages."""
        # Format coordinates to ensure consistent display
        lon_str = f"{longitude:.8f}"
        lat_str = f"{latitude:.8f}"
        
        # Update both pages to maintain consistency
        self.ui.lineEdit_ovv_longitude.setText(lon_str)
        self.ui.lineEdit_ovv_latitude.setText(lat_str)
        self.ui.lineEdit_sett_longitude.setText(lon_str)
        self.ui.lineEdit_sett_latitude.setText(lat_str)

    def _update_position_log(self, uav_index, latitude, longitude, altitude=0.0):
        """Update the current position log file for the UAV."""
        if latitude is None or longitude is None:
            return

        try:
            position_file = self.config.drone_current_pos_files[uav_index - 1]
            
            if not os.path.exists(os.path.dirname(position_file)):
                os.makedirs(os.path.dirname(position_file), exist_ok=True)
                
            with open(position_file, "w") as f:
                f.write(f"{latitude},{longitude}")
                
            # Only log history when the UAV is on a mission
            if self.UAVs[uav_index].telemetry.on_mission:
                mission_time = self.UAVs[uav_index].telemetry.mission_start_time if self.UAVs[uav_index].telemetry.mission_start_time else datetime.now().strftime("%Y%m%d")
                history_file = position_file.replace(".txt", f"_history_{mission_time}.txt")
                with open(history_file, "a") as f:
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    f.write(f"{timestamp},{latitude},{longitude},{altitude}\n")
                
        except Exception as e:
            self.logger.log(f"Failed to update position log for UAV {uav_index}: {e}", level="warning")

        


    #SEND COORDINATE ham gui tin nhan
    async def send_coordinate(self) -> None:
        # Ket noi cong com
        port = "/dev/ttyUSB0"
        baudrate = 9600
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            self.ui.mainTerminal.appendPlainText("Connected to " + port)

            # Gui tin nhan

            phone_number = "0972368553"
            script_dir = os.path.dirname(os.path.abspath(__file__))
            log_file_path = os.path.join(script_dir, "logs", "detected_pos", "detection_pos_uav_1.log")

            try:
                with open(log_file_path, "r", encoding="utf-8") as file:
                    message = file.readline().strip()
            except FileNotFoundError:
                self.ui.mainTerminal.appendPlainText("Error: File not found")
                return
            self.serial_port.write("AT+CMGF=1\n".encode())
            time.sleep(1)
            self.serial_port.write(f'AT+CMGS="{phone_number}"\n'.encode())
            time.sleep(1)
            self.serial_port.write((message + "\x1A").encode())
            time.sleep(3)
            response = self.serial_port.read_all().decode(errors='ignore')
            self.ui.mainTerminal.appendPlainText("Response: " + response)
        except serial.SerialException as e:
            self.ui.mainTerminal.appendPlainText("Error: " + str(e))






            
    def _check_uav_connection(self, uav_index, strictly=True):
        """Check if a UAV is connected and allowed to receive commands."""
        if strictly:
            return (self.UAVs[uav_index].telemetry.connected and 
                    self.UAVs[uav_index].config.connection_allow)
        else:
            return (self.UAVs[uav_index].telemetry.connected or
                    self.UAVs[uav_index].config.connection_allow)
            



    async def _process_detection_results(self, uav_index, annotated_frame, detected_results):
        """Process object detection results and handle detected targets."""
        
        if detected_results is None or len(detected_results) != 2:
            return
            
        track_ids, objects = detected_results
        
        for track_id, obj in zip(track_ids, objects):
            # Skip if not a detected person
            if not obj["detected"] or obj["class"] != "person":
                continue
            # Disable detection feature after finding a target
            await self.drone_service.pause_mission(uav_index)
            self.UAVs[uav_index].config.detection_enabled = False 
            self.UAVs[uav_index].telemetry.on_mission = False
            if self.view.active_tab_index == uav_index:
                self._set_pause_button_style("Resume")

            # Get UAV position and frame information
            frame_shape = annotated_frame.shape
            detected_pos = (obj["x"], obj["y"])
            
            # Get current GPS coordinates
            # with open(drone_current_pos_files[uav_index - 1], "r") as f:
            #     gps_data = f.read()
            #     uav_lat, uav_long = map(float, gps_data.split(","))
                
            uav_lat, uav_long = self.UAVs[uav_index].telemetry.latitude, self.UAVs[uav_index].telemetry.longitude
            uav_alt = self.UAVs[uav_index].telemetry.altitude_relative_m
            uav_gps = [uav_lat, uav_long, uav_alt]

            # Export detection to GPS log
            asyncio.create_task(export_points_to_gps_log(
                uav_index=uav_index,
                detected_pos=detected_pos,
                frame_shape=frame_shape,
                uav_gps=uav_gps,
            ))
            # Save the detection frame
            asyncio.create_task(self._save_detection_image(uav_index, track_id, annotated_frame))
            
            # Log detection to terminal
            asyncio.create_task(self._log_detection(uav_index, obj["class"], detected_pos, frame_shape, uav_gps))

            await asyncio.sleep(1)
            await self.drone_service.start_mission(uav_index)
            self.UAVs[uav_index].telemetry.on_mission = True
            if self.view.active_tab_index == uav_index:
                self._set_pause_button_style("Pause")
            await asyncio.sleep(25)
            self.UAVs[uav_index].config.detection_enabled = True

            # Only process the first detected person
            break

    async def _save_detection_image(self, uav_index, track_id, frame):
        """Save a detection image to the logs directory."""
        image_path = f"{__current_path__}/logs/images/UAV{uav_index}_locked_target_{track_id}.png"
        os.makedirs(os.path.dirname(image_path), exist_ok=True)
        cv2.imwrite(image_path, frame)
        self.logger.log(f"Saved detection image to {image_path}", level="info")

    async def _log_detection(self, uav_index, class_name, detected_pos, frame_shape, uav_gps):
        """Log detection information to the terminal."""
        
        detection_msg = (
            f"UAV-{uav_index} at GPS ({uav_gps[0]}, {uav_gps[1]}, {uav_gps[2]}m) "
            f"detected {class_name} at X: {detected_pos[0]:.1f} Y: {detected_pos[1]:.1f} "
            f"with frame size: {frame_shape[1]}x{frame_shape[0]}"
        )
        self.view.update_terminal(detection_msg, 0)
        self.logger.log(detection_msg, level="info")

    async def monitor_mission_progress(self, uav_index):
        """Monitor mission progress and update the UI progress bar for the given UAV."""
        try:
            progress_bars = [
                self.ui.progressUAV1_2, self.ui.progressUAV2_2, self.ui.progressUAV3_2,
                self.ui.progressUAV4_2, self.ui.progressUAV5_2, self.ui.progressUav6_2
            ]
            labels = [
                self.ui.progressLabel1_2, self.ui.progressLabel2_2, self.ui.progressLabel3_2,
                self.ui.progressLabel4_2, self.ui.progressLabel5_2, self.ui.progressLabel6_2
            ]
            
            if not (1 <= uav_index <= 6): return
                
            bar = progress_bars[uav_index - 1]
            label = labels[uav_index - 1]
            
            bar.setValue(0)
            label.setText("0/0")
            
            while True:
                try:
                    async for progress in self.drone_service.get_uav(uav_index).system.mission.mission_progress():
                        current = progress.current
                        total = progress.total
                        if total > 0:
                            bar.setMaximum(total)
                            bar.setValue(current)
                            if current == total:
                                label.setText(f"Finished ({total})")
                            else:
                                label.setText(f"{current}/{total}")
                except Exception:
                    pass
                await asyncio.sleep(1) # Tránh dùng float vì gây lỗi startTimer trong asyncqt
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.logger.error(f"Error monitoring mission progress for UAV {uav_index}: {e}")

    async def _track_uav_position_on_map(self, uav_index):
        """Continuously move the UAV marker while a mission is active."""
        try:
            uav = self.UAVs[uav_index]
            if uav.system is None:
                return

            last_update = 0.0
            async for position in uav.system.telemetry.position():
                if not uav.telemetry.on_mission:
                    break

                now = time.time()
                if now - last_update < 0.5:
                    continue
                last_update = now

                uav.telemetry.latitude = round(position.latitude_deg, 12)
                uav.telemetry.longitude = round(position.longitude_deg, 12)
                uav.telemetry.altitude_msl_m = round(position.absolute_altitude_m, 12)
                uav.telemetry.altitude_relative_m = round(position.relative_altitude_m, 12)

                self._update_position_log(
                    uav_index,
                    uav.telemetry.latitude,
                    uav.telemetry.longitude,
                    uav.telemetry.altitude_msl_m,
                )
                self.view.update_single_drone_position(
                    uav_index,
                    uav.telemetry.latitude,
                    uav.telemetry.longitude,
                )
                self.telemetry_controller._update_uav_info_display(uav_index)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.logger.log(f"Failed to track UAV {uav_index} position on map: {e}", level="warning")

    def _on_map_type_changed(self, index):
        """Đổi form nhập liệu tương ứng khi chọn Map Type."""
        if index in [0, 3, 4]:  # Square, Pentagon, Hexagon dùng chung trang 1 tham số
            self.ui.stackedMapParam_2.setCurrentIndex(0)
            if index == 0:
                self.ui.label_26.setText("Side Length (m)")
            else:
                self.ui.label_26.setText("Radius (m)")
        elif index == 1:  # Rectangle
            self.ui.stackedMapParam_2.setCurrentIndex(1)
        elif index == 2:  # Circle
            self.ui.stackedMapParam_2.setCurrentIndex(2)
        elif index in [5, 6]:  # Rescue, Custom
            self.ui.stackedMapParam_2.setCurrentIndex(3)

    # ------------------------------------< Simulation Functions >-----------------------------
    def _get_selected_sim_uavs(self):
        sim_uav_widgets = [
            self.ui.checkBox_sim_uav1,
            self.ui.checkBox_sim_uav2,
            self.ui.checkBox_sim_uav3,
            self.ui.checkBox_sim_uav4,
            self.ui.checkBox_sim_uav5,
            self.ui.checkBox_sim_uav6,
        ]

        if self.ui.All_UAVs.isChecked():
            return list(range(1, min(len(sim_uav_widgets), self.config.MAX_UAV_COUNT) + 1))

        return [
            index
            for index, widget in enumerate(sim_uav_widgets, start=1)
            if widget.isChecked()
        ]

    async def _get_sim_uav_start_positions(self, selected_uavs, use_current=True):
        start_positions = {}
        for uav_index in selected_uavs:
            if use_current:
                await self.telemetry_controller.uav_fn_get_position(uav_index)
                start_lat = self.UAVs[uav_index].telemetry.latitude
                start_lon = self.UAVs[uav_index].telemetry.longitude
            else:
                start_lat = None
                start_lon = None

            if not isinstance(start_lat, (int, float)) or not isinstance(start_lon, (int, float)):
                if use_current:
                    self.view.update_terminal(
                        f"[SIM] Không lấy được vị trí hiện tại UAV {uav_index}, dùng vị trí khởi tạo.",
                        0,
                    )
                else:
                    self.view.update_terminal(
                        f"[SIM] Planning-only: dùng vị trí khởi tạo của UAV {uav_index}.",
                        0,
                    )
                start_lat = self.UAVs[uav_index].config.init_params["latitude"]
                start_lon = self.UAVs[uav_index].config.init_params["longitude"]

            start_positions[uav_index] = (start_lat, start_lon)
        return start_positions

    def _ensure_path_starts_at_uav(self, path, start_coord):
        if not path:
            return []
        if abs(path[0][0] - start_coord[0]) > 1e-7 or abs(path[0][1] - start_coord[1]) > 1e-7:
            return [start_coord] + path
        return path

    def _load_mission_energy_model(self, target, model_name=MISSION_ENERGY_MODEL_NAME):
        cache_key = (target, model_name)
        if cache_key in self._mission_energy_models:
            return self._mission_energy_models[cache_key]

        model_path = MISSION_ENERGY_MODEL_DIR / f"{target}_{model_name}.pkl"
        with model_path.open("rb") as model_file:
            model = pickle.load(model_file)
        self._mission_energy_models[cache_key] = model
        return model

    def _predict_single_path_energy_time(self, distance_m, waypoint_count):
        features = pd.DataFrame([{
            "total_distance_m": max(float(distance_m), 0.0),
            "stop_count": max(int(waypoint_count), 1),
        }], columns=MISSION_ENERGY_FEATURE_COLUMNS)

        energy_model = self._load_mission_energy_model("total_flight_energy_wh")
        time_model = self._load_mission_energy_model("total_flight_time_s")
        energy_wh = float(energy_model.predict(features)[0])
        flight_time_s = float(time_model.predict(features)[0])
        return max(energy_wh, 0.0), max(flight_time_s, 0.0)

    def _predict_planning_only_energy_time(self, paths_by_uav, distances_by_uav):
        total_energy_wh = 0.0
        longest_flight_time_s = 0.0
        for uav_index, path in paths_by_uav.items():
            waypoint_count = max(len(path) - 1, 1)
            energy_wh, flight_time_s = self._predict_single_path_energy_time(
                distances_by_uav.get(uav_index, 0.0),
                waypoint_count,
            )
            total_energy_wh += energy_wh
            longest_flight_time_s = max(longest_flight_time_s, flight_time_s)
        return total_energy_wh, longest_flight_time_s

    def _clear_simulation_layers(self, include_grid=False, include_best=False):
        marker_prefixes = ["sim_current_pt_"]
        polyline_names = ["current_run_path_polyline"]

        if include_grid:
            marker_prefixes.append("sim_grid_pt_")
        if include_best:
            marker_prefixes.append("sim_best_pt_")
            polyline_names.append("best_path_polyline")

        marker_prefixes_js = ", ".join(repr(prefix) for prefix in marker_prefixes)
        polyline_names_js = ", ".join(repr(name) for name in polyline_names)
        self.view.sim_map.runScript(f"""
            if (typeof map !== 'undefined') {{
                const markerPrefixes = [{marker_prefixes_js}];
                const polylineNames = [{polyline_names_js}];
                map.eachLayer(function (layer) {{
                    if (layer instanceof L.Marker && layer.options.name) {{
                        if (markerPrefixes.some(function(prefix) {{ return layer.options.name.startsWith(prefix); }})) {{
                            map.removeLayer(layer);
                        }}
                    }}
                    if (layer instanceof L.Polyline && layer.options.name) {{
                        if (polylineNames.indexOf(layer.options.name) !== -1) {{
                            map.removeLayer(layer);
                        }}
                    }}
                }});
            }}
        """)

    async def run_simulation_callback(self):
        """
        Handles the 'Run Simulation' button click event.
        - Generates a mission area based on UI settings.
        - Creates grid points within that area.
        - Runs selected pathfinding algorithms.
        - Displays results in the comparison table and on the simulation map.
        """

        selected_uavs = self._get_selected_sim_uavs()
        if not selected_uavs:
            self.view.popup_msg("Vui lòng chọn ít nhất một UAV để chạy Simulation.", "Simulation", "Warning")
            return

        map_type = self.ui.comboBox_3.currentText()
        num_runs = self.ui.Num_run_2.value()
        planning_only = num_runs == 0
        if num_runs < 0:
            self.view.update_terminal("[SIM] Simulation aborted: Runs per Algorithm must be 0 or greater.", 0)
            self.view.popup_msg("Runs per Algorithm phải lớn hơn hoặc bằng 0.", "Simulation", "Warning")
            return
        run_iterations = 1 if planning_only else num_runs

        disconnected_uavs = []
        if not planning_only:
            disconnected_uavs = [
                uav_index
                for uav_index in selected_uavs
                if not self._check_uav_connection(uav_index, strictly=True)
            ]
        if disconnected_uavs:
            uav_names = ", ".join(f"UAV {uav_index}" for uav_index in disconnected_uavs)
            self.view.update_terminal(f"[SIM] {uav_names} chưa connected. Simulation aborted.", 0)
            self.view.popup_msg(f"Vui lòng Connect {uav_names} trước khi chạy Simulation!", "Simulation", "Warning")
            return

        if planning_only:
            self.view.update_terminal("[SIM] Planning-only mode: drawing path planning without UAV connection or flight.", 0)
        else:
            self.view.update_terminal("[SIM] Starting simulation...", 0)

        # 1. Clear only temporary simulation layers. Keep existing labels/areas/path
        # visible until their replacements are ready to draw.
        self._clear_simulation_layers()

        # Clear only data columns, keep algorithm names in column 0
        for row in range(self.ui.tableWidgetAlgorithmComparison.rowCount()):
            for col in range(1, self.ui.tableWidgetAlgorithmComparison.columnCount()):
                self.ui.tableWidgetAlgorithmComparison.setItem(row, col, QtWidgets.QTableWidgetItem(""))
        try:
            grid_size = float(self.ui.gridSize_line_edit.text()) # Lấy tạm grid size từ tab map
            if grid_size <= 0: grid_size = 10.0
        except ValueError:
            grid_size = 10.0

        # Lấy danh sách thuật toán được chọn
        selected_algos = []
        if self.ui.zigzag_2.isChecked(): selected_algos.append("Zigzag")
        if self.ui.FindPath_2.isChecked(): selected_algos.append("Find_Path")
        if self.ui.NN2Opt_2.isChecked(): selected_algos.append("NN_2opt")
        if self.ui.SA_2.isChecked(): selected_algos.append("SA")
        if self.ui.ACO_2.isChecked(): selected_algos.append("ACO")
        if self.ui.GA_2.isChecked(): selected_algos.append("GA")
        if self.ui.GA_with_1.isChecked(): selected_algos.append("GA_with_turn")
        if self.ui.ABC_2.isChecked(): selected_algos.append("ABC")
        if self.ui.Improve_A_2.isChecked(): selected_algos.append("A*_Improved")

        # Lấy trạng thái của checkbox reduce_points
        reduce_points_enabled = self.ui.Reduce_Points.isChecked()

        if not selected_algos:
            self.view.popup_msg("Please select at least one algorithm.", "Simulation", "Warning")
            return

        total_runs = len(selected_algos) * run_iterations
        current_run = 0
        self.ui.progressBar.setMaximum(total_runs)
        self.ui.progressBar.setValue(0)
        self.ui.label_32.setText(f"0/{total_runs}")
        await asyncio.sleep(0) # Ép giao diện render ngay lập tức trạng thái 0/x trước khi thuật toán chạy

        start_positions = await self._get_sim_uav_start_positions(selected_uavs, use_current=not planning_only)
        center_lat = sum(pos[0] for pos in start_positions.values()) / len(start_positions)
        center_lon = sum(pos[1] for pos in start_positions.values()) / len(start_positions)
        center_coord = (center_lat, center_lon)
        self.view.update_terminal(
            f"[SIM] Selected UAVs: {', '.join(f'UAV {uav}' for uav in selected_uavs)}. "
            f"Splitting area into {len(selected_uavs)} parts.",
            0,
        )

        # 3. Generate mission area (polygon)
        try:
            # Focus bản đồ mô phỏng vào khu vực này
            self.view.sim_map.setZoom(18)

            if map_type == "Square":
                side = self.ui.spinBox_5.value()
                if side <= 0: side = 100.0  # Mặc định 100m
                # Tạo 4 góc của hình vuông
                half_side = side / 2.0
                p1 = calculate_new_lat_lon(center_lat, center_lon, half_side, -half_side)  # Top-left
                p2 = calculate_new_lat_lon(center_lat, center_lon, half_side, half_side)   # Top-right
                p3 = calculate_new_lat_lon(center_lat, center_lon, -half_side, half_side)  # Bottom-right
                p4 = calculate_new_lat_lon(center_lat, center_lon, -half_side, -half_side) # Bottom-left
                polygon_vertices = [p1, p2, p3, p4, p1] # Dùng để vẽ trên bản đồ
            elif map_type == "Rectangle":
                width = self.ui.spinBox_6.value()
                height = self.ui.spinBox_7.value()
                if width <= 0: width = 100.0
                if height <= 0: height = 100.0
                half_w, half_h = width / 2.0, height / 2.0
                p1 = calculate_new_lat_lon(center_lat, center_lon, half_h, -half_w) # Top-left
                p2 = calculate_new_lat_lon(center_lat, center_lon, half_h, half_w)  # Top-right
                p3 = calculate_new_lat_lon(center_lat, center_lon, -half_h, half_w) # Bottom-right
                p4 = calculate_new_lat_lon(center_lat, center_lon, -half_h, -half_w)# Bottom-left
                polygon_vertices = [p1, p2, p3, p4, p1] # Dùng để vẽ trên bản đồ
            elif map_type == "Custom":
                custom_polygons = self.view.geodata.get("Polygon", [])
                if not custom_polygons:
                    self.view.popup_msg(
                        "Vui lòng vẽ một vùng Custom trên Rescue Map trước khi chạy Simulation.",
                        "Simulation",
                        "Warning",
                    )
                    self.view.update_terminal("[SIM] Simulation aborted: No custom polygon selected.", 0)
                    return

                polygon_vertices = [
                    (float(lat), float(lon))
                    for lat, lon in custom_polygons[-1]
                ]
                if len(polygon_vertices) < 3:
                    self.view.popup_msg("Vùng Custom cần ít nhất 3 điểm.", "Simulation", "Warning")
                    self.view.update_terminal("[SIM] Simulation aborted: Invalid custom polygon.", 0)
                    return

                polygon_points_for_center = (
                    polygon_vertices[:-1]
                    if polygon_vertices[0] == polygon_vertices[-1]
                    else polygon_vertices
                )
                center_lat = sum(point[0] for point in polygon_points_for_center) / len(polygon_points_for_center)
                center_lon = sum(point[1] for point in polygon_points_for_center) / len(polygon_points_for_center)

                if polygon_vertices[0] != polygon_vertices[-1]:
                    polygon_vertices.append(polygon_vertices[0])

            else:
                self.view.popup_msg(f"Map Type '{map_type}' is not implemented yet.", "Simulation", "Warning")
                return

            self.view.sim_map.centerAt(center_lat, center_lon)

            split_result = self.view.split_area_for_mission(
                polygon_vertices,
                len(selected_uavs),
                map_names=("sim",),
                clear_existing=False,
                store=False,
                save_files=False,
            )

            # VẼ LẠI DRONE LÊN BẢN ĐỒ SIMULATION SAU KHI BỊ XÓA
            self.view.update_drone_positions()

            if len(selected_uavs) == 1:
                ref_lat, ref_lon = center_coord
                cartesian_poly = [latlon_to_xy(ref_lat, ref_lon, lat, lon) for lat, lon in polygon_vertices]
                grid_points_cartesian = generate_grid(cartesian_poly, grid_size)
                grid_points_latlon = [
                    calculate_new_lat_lon(ref_lat, ref_lon, p[1], p[0])
                    for p in grid_points_cartesian
                ]
                grid_point_groups = [grid_points_latlon]
            else:
                grid_point_groups = split_grids(
                    split_result["rotated_area_list"],
                    *split_result["extra"],
                    grid_size,
                    len(selected_uavs),
                )

            if not grid_point_groups:
                self.view.popup_msg("Vùng sinh quá bé hoặc Grid Size quá lớn, không đủ tạo điểm bay!", "Simulation", "Warning")
                self.view.update_terminal("[SIM] Simulation aborted: Not enough points.", 0)
                return

            if len(grid_point_groups) != len(selected_uavs):
                self.view.popup_msg(
                    "Không chia được đủ area cho số UAV đã chọn.",
                    "Simulation",
                    "Warning",
                )
                self.view.update_terminal(
                    f"[SIM] Simulation aborted: expected {len(selected_uavs)} grid groups, got {len(grid_point_groups)}.",
                    0,
                )
                return

            grid_points_by_uav = {
                uav_index: grid_point_groups[ind]
                for ind, uav_index in enumerate(selected_uavs)
            }
            empty_uavs = [
                uav_index
                for uav_index, points in grid_points_by_uav.items()
                if not points
            ]
            if empty_uavs:
                self.view.popup_msg(
                    "Grid Size quá lớn hoặc vùng quá nhỏ, một số UAV không có điểm bay.",
                    "Simulation",
                    "Warning",
                )
                self.view.update_terminal(
                    f"[SIM] Simulation aborted: no grid points for {empty_uavs}.",
                    0,
                )
                return

            for uav_index, points in grid_points_by_uav.items():
                for i, point in enumerate(points):
                    marker_options = {
                        'icon': str(DOT_ICON_PATH),
                        'iconSize': {'width': 5, 'height': 5},
                        'title': f'UAV {uav_index} Area Point {i + 1}',
                        'name': 'sim_grid_pt_'
                    }
                    self.view.sim_map.addMarker(
                        f"sim_grid_pt_uav{uav_index}_{i + 1}",
                        point[0],
                        point[1],
                        **marker_options,
                    )

        except Exception as e:
            self.view.popup_msg(f"Error generating map area/grid: {e}", "Simulation", "Error")
            self.logger.error(f"[SIM] Error generating map/grid: {e}")
            return

        # 5. Run algorithms and display results
        if planning_only:
            self.view.update_terminal(f"[SIM] Planning paths for {len(selected_algos)} algorithms...", 0)
        else:
            self.view.update_terminal(f"[SIM] Running {len(selected_algos)} algorithms, {num_runs} runs each...", 0)
        await asyncio.sleep(0) # Nhường quyền cho event loop in log ra màn hình trước

        algo_map = {
            "Zigzag": lambda pts, start: find_zigzag_path(pts, start)[0],
            "Find_Path": find_path_0,
            "NN_2opt": nn_2opt_path,
            "SA": sa_path,
            "ACO": aco_path,
            "GA": ga_path,
            "GA_with_turn": ga_path_with_turns,
            "A*_Improved": astar_path_with_turns,
            "ABC": abc_path
        }

        algo_to_row = {
            "Zigzag": 0,
            "Find_Path": 1,
            "NN_2opt": 2,
            "SA": 3,
            "ACO": 4,
            "GA": 5,
            "GA_with_turn": 6,
            "ABC": 7,
            "A*_Improved": 8
        }

        best_overall_score = float('inf')
        best_overall_path = None
        best_overall_algo = ""

        for algo_name in selected_algos:
            if algo_name not in algo_map:
                continue

            # Thêm total_overlap_ratio để tính toán giá trị mới
            total_cost, total_dist, total_turns, total_swept, total_coverage, total_overlap_ratio = 0, 0, 0, 0, 0, 0
            total_flight_time = 0
            total_energy_wh = 0
            current_best_path_for_algo = []

            try:
                for run_idx in range(run_iterations): # type: ignore
                    if planning_only:
                        self.view.update_terminal(f"\n[SIM] === Planning {algo_name.replace('_', ' ')} ===", 0)
                    else:
                        self.view.update_terminal(f"\n[SIM] === Đang chạy {algo_name.replace('_', ' ')} - Lượt {run_idx+1}/{num_runs} ===", 0)
                    await asyncio.sleep(0)

                    # 1. TÍNH TOÁN ĐƯỜNG ĐI TOÁN HỌC CHO TỪNG UAV ĐƯỢC CHỌN
                    paths_by_uav = {}
                    distances_by_uav = {}
                    run_cost = 0
                    run_dist = 0
                    run_turns = 0
                    run_swept = 0
                    run_coverage = 0
                    run_overlap_ratio = 0

                    for uav_index in selected_uavs:
                        uav_area_points = grid_points_by_uav[uav_index]
                        start_coord = start_positions[uav_index]
                        path = algo_map[algo_name](uav_area_points.copy(), start_coord)
                        if not path:
                            self.view.update_terminal(
                                f"[SIM] Thuật toán {algo_name} không trả về đường đi cho UAV {uav_index}. Bỏ qua lượt này.",
                                0,
                            )
                            paths_by_uav = {}
                            break

                        path = self._ensure_path_starts_at_uav(path, start_coord)

                        if reduce_points_enabled:
                            original_point_count = len(path)
                            path_start = path[0]
                            reduced_grid_points = self.view.reduce_points_in_path(path[1:])
                            path = [path_start] + reduced_grid_points
                            reduced_point_count = len(path)
                            self.view.update_terminal(
                                f"[SIM] UAV {uav_index}: reduced path for {algo_name} "
                                f"from {original_point_count} to {reduced_point_count} waypoints.",
                                0,
                            )

                        analyzer = UAVAnalyzer(
                            area_gps=polygon_vertices,
                            flight_path=path,
                            footprint_size=20.0,
                        )
                        analysis_result = analyzer.compute_coverage()

                        run_cost += analysis_result.get("cost", 0)
                        path_distance_m = analysis_result.get("distance_m", 0)
                        run_dist += path_distance_m
                        run_turns += analysis_result.get("turns", 0)
                        run_swept += analysis_result.get("swept_area_m2", 0)
                        run_coverage += analysis_result.get("coverage_percent", 0) / 100.0
                        paths_by_uav[uav_index] = path
                        distances_by_uav[uav_index] = path_distance_m

                    if not paths_by_uav:
                        current_run += 1
                        self.ui.progressBar.setValue(current_run)
                        self.ui.label_32.setText(f"{current_run}/{total_runs}")
                        await asyncio.sleep(0)
                        continue

                    run_coverage = min(run_coverage, 1.0)
                    overlap_result = analyzer.compute_overlap_for_paths(list(paths_by_uav.values()))
                    run_overlap_ratio = overlap_result.get("overlap_ratio", 0)

                    total_cost += run_cost
                    total_dist += run_dist
                    total_turns += run_turns
                    total_swept += run_swept
                    total_coverage += run_coverage
                    total_overlap_ratio += run_overlap_ratio
                    current_best_path_for_algo = paths_by_uav

                    # --- DRAWING CURRENT PATH (MARKERS & POLYLINE) BEFORE SIM ---
                    self._clear_simulation_layers(include_grid=True)
                    # Draw current path markers, skipping the first point (UAV's start position)
                    # to avoid creating a "Point 0" marker. The polyline will connect from the UAV icon.
                    colors = ["orange", "red", "green", "blue", "purple", "brown"]
                    plan_files_by_uav = {}
                    for color_idx, uav_index in enumerate(selected_uavs):
                        path = paths_by_uav[uav_index]
                        for i, p in enumerate(path[1:]):
                            marker_options = {
                                'icon': str(DOT_ICON_PATH),
                                'iconSize': {'width': 5, 'height': 5},
                                'title': f'UAV {uav_index} - Point {i+1}',
                                'name': 'sim_current_pt_'
                            }
                            self.view.sim_map.addMarker(
                                f"sim_current_pt_uav{uav_index}_{i+1}",
                                p[0],
                                p[1],
                                **marker_options,
                            )

                        self.view.sim_map.drawPolyLine(
                            f"current_run_path_polyline_uav{uav_index}",
                            path,
                            options={
                                'color': colors[color_idx % len(colors)],
                                'weight': 3,
                                'opacity': 0.7,
                                'name': 'current_run_path_polyline',
                            },
                        )

                        sim_plan_file = os.path.join(
                            __current_path__,
                            "logs",
                            "points",
                            f"simulation_path_uav{uav_index}.txt",
                        )
                        os.makedirs(os.path.dirname(sim_plan_file), exist_ok=True)
                        uav_alt = self.UAVs[uav_index].config.init_params.get("altitude", 10.0)
                        with open(sim_plan_file, "w") as f:
                            for pt in path:
                                f.write(f"{pt[0]},{pt[1]},{uav_alt}\n")
                        plan_files_by_uav[uav_index] = sim_plan_file

                    if planning_only:
                        try:
                            energy_wh, flight_time = self._predict_planning_only_energy_time(
                                paths_by_uav,
                                distances_by_uav,
                            )
                            self.view.update_terminal(
                                f"[SIM] Planning-only: drew {len(paths_by_uav)} UAV paths for {algo_name}. "
                                f"Predicted Time = {flight_time:.1f}s, Energy = {energy_wh:.3f} Wh "
                                f"({MISSION_ENERGY_MODEL_NAME} model).",
                                0,
                            )
                        except Exception as e:
                            flight_time = 0
                            energy_wh = 0
                            self.view.update_terminal(
                                f"[SIM] Planning-only: drew {len(paths_by_uav)} UAV paths for {algo_name}, "
                                f"but energy/time prediction failed: {e}",
                                0,
                            )
                            self.logger.error(f"[SIM] Mission energy/time prediction failed: {e}")
                    else:
                        # 3. ĐO LƯỜNG VÀ BAY MÔ PHỎNG THỰC TẾ
                        # Nạp mission và chạy đồng thời cho tất cả UAV được chọn.
                        start_time = time.time()
                        sim_flight_tasks = [
                            asyncio.create_task(self._run_single_sim_flight(uav_index, sim_plan_file))
                            for uav_index, sim_plan_file in plan_files_by_uav.items()
                        ]
                        sim_flight_results = await asyncio.gather(*sim_flight_tasks, return_exceptions=True)
                        for result in sim_flight_results:
                            if isinstance(result, Exception):
                                raise result
                        flight_time = time.time() - start_time

                        measured_energy = [result for result in sim_flight_results if result is not None]
                        missing_measurements = len(sim_flight_results) - len(measured_energy)
                        energy_wh = sum(measured_energy)
                        if missing_measurements:
                            # Only used if PX4 has not supplied valid voltage/current samples.
                            # 120 W is a conservative x500 fallback; normal runs use measured V*I.
                            fallback_wh = missing_measurements * 120.0 * flight_time / 3600.0
                            energy_wh += fallback_wh
                            self.view.update_terminal(
                                f"[SIM] Cảnh báo: thiếu dữ liệu V/I của {missing_measurements} UAV; "
                                f"dùng tải dự phòng 120 W ({fallback_wh:.3f} Wh).",
                                0,
                            )

                        # --- CLEARING CURRENT PATH DRAWINGS AFTER SIM ---
                        self._clear_simulation_layers()
                    total_flight_time += flight_time
                    total_energy_wh += energy_wh
                    if not planning_only:
                        self.view.update_terminal(
                            f"[SIM] Kết quả lượt {run_idx+1}: {len(paths_by_uav)} UAV paths, "
                            f"Thời gian = {flight_time:.1f}s, Năng lượng = {energy_wh:.3f} Wh",
                            0,
                        )
                    # 4. CẬP NHẬT GIAO DIỆN
                    current_run += 1
                    self.ui.progressBar.setValue(current_run)
                    self.ui.label_32.setText(f"{current_run}/{total_runs}")
                    await asyncio.sleep(0)
                    if current_run < total_runs and not planning_only:
                        self.view.update_terminal("[SIM] Đợi 5 giây làm nguội trước khi cất cánh lượt tiếp theo...", 0)
                        await asyncio.sleep(5)

                # 5. TÍNH TRUNG BÌNH & ĐIỀN VÀO BẢNG TỔNG KẾT
                if run_iterations > 0:
                    avg_cost = total_cost / run_iterations
                    avg_dist = total_dist / run_iterations
                    avg_turns = total_turns / run_iterations
                    avg_swept = total_swept / run_iterations
                    avg_time = total_flight_time / run_iterations
                    avg_energy_wh = total_energy_wh / run_iterations
                    avg_coverage = total_coverage / run_iterations
                    avg_overlap_ratio = total_overlap_ratio / run_iterations

                    row = algo_to_row.get(algo_name)
                    if row is not None:
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{avg_cost:.2f}"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{avg_coverage:.3f}"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{avg_swept:.2f} m²"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 4, QtWidgets.QTableWidgetItem(f"{avg_energy_wh:.3f} Wh"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 5, QtWidgets.QTableWidgetItem(f"{avg_time:.1f} s"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 6, QtWidgets.QTableWidgetItem(f"{avg_dist:.2f} m"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 7, QtWidgets.QTableWidgetItem(f"{avg_overlap_ratio * 100:.2f}%"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 8, QtWidgets.QTableWidgetItem(f"{avg_turns:.1f}"))
                        
                        # Score thấp hơn là tốt hơn; năng lượng là tổng Wh của cả đội UAV.
                        score = avg_cost + (avg_time * 0.1) + (avg_energy_wh * 10)
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 9, QtWidgets.QTableWidgetItem(f"{score:.1f}"))
                        
                        # Cập nhật xem thuật toán nào đang vô địch về Score
                        if score < best_overall_score:
                            best_overall_score = score
                            best_overall_algo = algo_name
                            best_overall_path = current_best_path_for_algo
            except Exception as e:
                self.view.update_terminal(f"[SIM] Lỗi thuật toán {algo_name}: {e}", 0)
                self.logger.error(f"[SIM] Error in algorithm {algo_name}: {e}")
                continue

        # 6. HOÀN TẤT & VẼ KẾT QUẢ TỐT NHẤT LÊN BẢN ĐỒ
        self._clear_simulation_layers(include_best=True)

        if best_overall_path:
            colors = ["purple", "red", "green", "blue", "orange", "brown"]
            for color_idx, (uav_index, path) in enumerate(best_overall_path.items()):
                for i, p in enumerate(path):
                    marker_options = {
                        'icon': str(DOT_ICON_PATH),
                        'iconSize': {'width': 5, 'height': 5},
                        'title': f'UAV {uav_index} - Point {i}',
                        'name': 'sim_best_pt_'
                    }
                    self.view.sim_map.addMarker(
                        f"sim_best_pt_uav{uav_index}_{i}",
                        p[0],
                        p[1],
                        **marker_options,
                    )

                self.view.sim_map.drawPolyLine(
                    f"best_path_uav{uav_index}",
                    path,
                    options={
                        'color': colors[color_idx % len(colors)],
                        'weight': 5,
                        'name': 'best_path_polyline',
                    },
                )

            if planning_only:
                self.view.update_terminal(f"\n[SIM] === HOÀN TẤT PATH PLANNING ===", 0)
                self.view.update_terminal(f"[SIM] Path planning tốt nhất: {best_overall_algo.replace('_', ' ')} (Score: {best_overall_score:.1f})", 0)
            else:
                self.view.update_terminal(f"\n[SIM] === HOÀN TẤT MÔ PHỎNG MỌI THUẬT TOÁN ===", 0)
                self.view.update_terminal(f"[SIM] Thuật toán tốt nhất thực tế: {best_overall_algo.replace('_', ' ')} (Score: {best_overall_score:.1f})", 0)

            # --- THÊM VÀO ĐỂ LƯU VÀ MỞ ẢNH KẾT QUẢ ---
            try:
                self.view.update_terminal("[SIM] Generating and saving coverage analysis image...", 0)
                # Tạo tên file ảnh duy nhất theo thời gian
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                image_path = os.path.join(__current_path__, "logs", "images", f"sim_result_{best_overall_algo}_{timestamp}.png")
                os.makedirs(os.path.dirname(image_path), exist_ok=True)

                final_path = next(iter(best_overall_path.values()))
                final_analyzer = UAVAnalyzer(
                    area_gps=polygon_vertices,
                    flight_path=final_path,
                    footprint_size=20.0
                )

                if len(best_overall_path) == 1:
                    final_analyzer.visualize(
                        title=f"Coverage Analysis - {best_overall_algo}",
                        save_path=image_path,
                        show=False
                    )
                else:
                    final_analyzer.visualize_multi_paths(
                        paths_by_label=best_overall_path,
                        title=f"Multi-UAV Coverage Analysis - {best_overall_algo}",
                        save_path=image_path,
                        show=False
                    )

                self.view.update_terminal(f"[SIM] Saved analysis image to {os.path.relpath(image_path)}", 0)
                # Tự động mở file ảnh
                if sys.platform == "win32":
                    os.startfile(image_path)
                else:
                    opener = "open" if sys.platform == "darwin" else "xdg-open"
                    subprocess.run([opener, image_path])
            except Exception as e:
                self.logger.error(f"[SIM] Failed to generate or open analysis image: {e}")
                self.view.update_terminal(f"[SIM] Error generating analysis image: {e}", 0)
            # --- KẾT THÚC PHẦN THÊM VÀO ---

            if planning_only:
                self.view.popup_msg(f"Path planning hoàn tất!\nThuật toán tối ưu nhất: {best_overall_algo.replace('_', ' ')}", "Simulation", "Info")
            else:
                self.view.popup_msg(f"Mô phỏng hoàn tất!\nThuật toán tối ưu nhất: {best_overall_algo.replace('_', ' ')}", "Simulation", "Info")
        else:
            if planning_only:
                self.view.update_terminal("[SIM] Hoàn tất path planning nhưng không tìm thấy đường đi hợp lệ.", 0)
            else:
                self.view.update_terminal("[SIM] Hoàn tất mô phỏng nhưng không tìm thấy đường đi hợp lệ.", 0)
    async def _run_single_sim_flight(self, uav_index, plan_file):
        background_tasks = []
        energy_wh = 0.0
        valid_energy_intervals = 0

        try:
            self.UAVs[uav_index].telemetry.on_mission = True
            self.UAVs[uav_index].telemetry.mission_start_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            if self.view.active_tab_index == uav_index:
                self._set_pause_button_style("Pause")
                
            # Khởi chạy progress bar bay
            progress_task = asyncio.create_task(self.monitor_mission_progress(uav_index))
            position_task = asyncio.create_task(self._track_uav_position_on_map(uav_index))

            try:
                await self.UAVs[uav_index].system.telemetry.set_rate_battery(10.0)
            except Exception as e:
                self.logger.log(f"[SIM] UAV {uav_index}: cannot set battery rate: {e}", level="warning")

            async def integrate_battery_energy():
                nonlocal energy_wh, valid_energy_intervals
                last_sample_time = None
                async for battery in self.UAVs[uav_index].system.telemetry.battery():
                    now = time.monotonic()
                    voltage_v = getattr(battery, "voltage_v", float("nan"))
                    current_a = getattr(battery, "current_battery_a", float("nan"))

                    telemetry = self.UAVs[uav_index].telemetry
                    telemetry.battery_voltage_v = voltage_v
                    telemetry.battery_current_a = current_a
                    telemetry.battery_consumed_ah = getattr(battery, "capacity_consumed_ah", None)
                    remaining = getattr(battery, "remaining_percent", float("nan"))
                    if np.isfinite(remaining):
                        telemetry.battery_percent = f"{remaining * 100:.1f}%"

                    if np.isfinite(voltage_v) and np.isfinite(current_a) and voltage_v > 0 and current_a >= 0:
                        if last_sample_time is not None:
                            dt_s = min(max(now - last_sample_time, 0.0), 2.0)
                            energy_wh += voltage_v * current_a * dt_s / 3600.0
                            valid_energy_intervals += 1
                        last_sample_time = now
                    else:
                        last_sample_time = None

            battery_task = asyncio.create_task(integrate_battery_energy())
            
            # Task ngầm: Liên tục kiểm tra xem bay hết điểm chưa để bắn lệnh về
            async def auto_rtl_when_finished():
                while self.UAVs[uav_index].telemetry.on_mission:
                    try:
                        if await self.UAVs[uav_index].system.mission.is_mission_finished():
                            self.view.update_terminal(f"[SIM] UAV {uav_index} hoàn tất các điểm, đang tự động quay về (RTL)...", 0)
                            await self.UAVs[uav_index].system.action.return_to_launch()
                            break
                    except Exception:
                        pass
                    await asyncio.sleep(1)
            
            rtl_task = asyncio.create_task(auto_rtl_when_finished())
            background_tasks = [rtl_task, progress_task, position_task, battery_task]
            
            # Kích hoạt hàm cất cánh và bay (hàm uav_fn_do_mission sẽ tự động block cho đến khi UAV đáp đất hoàn toàn)
            self.view.update_terminal(f"[SIM] Bắt đầu nạp lộ trình và cất cánh UAV {uav_index}...", 0)
            await self.drone_service.uav_fn_do_mission(uav_index=uav_index, mission_plan_file=plan_file)
            
            # Hủy các task ngầm khi chuyến bay đã kết thúc và đợi chúng cleanup.
            for task in background_tasks:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*background_tasks, return_exceptions=True)
            
            # Khôi phục nút UI
            self.UAVs[uav_index].telemetry.on_mission = False
            if self.view.active_tab_index == uav_index:
                self._set_pause_button_style("Resume")
                
            # QUAN TRỌNG: Đợi UAV xả động cơ (Disarm) hoàn toàn để reset hệ thống trước vòng lặp sau
            self.view.update_terminal(f"[SIM] Đợi UAV {uav_index} xả động cơ (Disarm) an toàn...", 0)
            while True:
                is_armed = False
                try:
                    async for armed in self.UAVs[uav_index].system.telemetry.armed():
                        is_armed = armed
                        break
                except Exception:
                    pass
                if not is_armed:
                    break
                await asyncio.sleep(1)

            return energy_wh if valid_energy_intervals > 0 else None
                
        except Exception as e:
            self.view.update_terminal(f"[SIM] Lỗi trong chuyến bay: {e}", 0)
            raise
        finally:
            for task in background_tasks:
                if not task.done():
                    task.cancel()
            if background_tasks:
                await asyncio.gather(*background_tasks, return_exceptions=True)
            self.UAVs[uav_index].telemetry.on_mission = False
        
    # ------------------------------------< Rescue UAV 6 >-----------------------------
    # ? developing ...
    async def uav_fn_rescue(self) -> None:
        if not ( # type: ignore
            self.UAVs[self.config.RESCUE_UAV_INDEX].telemetry.connected
            and self.UAVs[self.config.RESCUE_UAV_INDEX].config.connection_allow
        ):
            return

        self.view.update_terminal(f"[INFO] Sent RESCUE command to UAV {self.config.RESCUE_UAV_INDEX}")

        await self.drone_service.connect(self.config.RESCUE_UAV_INDEX)

        # check health 
        # TODO: check battery level here
        async for health in self.drone_service.get_uav(self.config.RESCUE_UAV_INDEX).system.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok: # type: ignore
                self.logger.log(
                    f"UAV-{RESCUE_UAV_INDEX} -- Global position for estimate OK", level="info"
                ) # type: ignore
                break
            
        self.logger.log(f"UAV-{RESCUE_UAV_INDEX} -- Arming and taking off", level="info")
        
        try:
            # do the rescue mission loop
            # 1. check if the rescue position is available
            # 2. get the detected UAVs
            # 3. do the rescue mission

            while True:
                # 1 check if rescue position is available
                rescue_filepaths = glob.glob(
                    f"{__current_path__}/logs/rescue_pos/rescue_pos_uav_*.log"
                )

                if len(rescue_filepaths) == 0:
                    await asyncio.sleep(1)
                    continue

                # NOTE: you can implement your own logic here
                # get the detected UAVs
                #detected_uav_list = []
                for rescue_filepath in rescue_filepaths:
                    uav_index = int(str(Path(rescue_filepath).stem).split("_")[-1])
                    print(f"Detected UAV: {uav_index}")
                    #detected_uav_list.append(self.UAVs[uav_index])
                
                # get the rescue filepath
                
                self.logger.log(
                    f"Found {len(rescue_filepaths)} rescue files",
                    level="info",
                )
                rescue_filepath = select_mission_plan(rescue_filepaths)
                self.logger.log(
                    f"Selected rescue file: {rescue_filepath}",
                    level="info",
                )
                
                # 
                self.logger.log("Rescue mission started...", level="info")

                await asyncio.sleep(1)
                await self.drone_service.takeoff(self.config.RESCUE_UAV_INDEX)
                await asyncio.sleep(10)
                               
                self.logger.log(
                    f"UAV-{RESCUE_UAV_INDEX} -- Takeoff completed, ready to start rescue mission", level="info"
                )

                # get initial position
                async for position in self.UAVs[RESCUE_UAV_INDEX].system.telemetry.position():
                    self.UAVs[RESCUE_UAV_INDEX].config.init_params["latitude"] = round(position.latitude_deg, 12)
                    self.UAVs[RESCUE_UAV_INDEX].config.init_params["longitude"] = round(position.longitude_deg, 12)
                    break
                
                # 2 UAV Rescue do the rescue mission and the detected drones goes into suspend mode
                self.UAVs[RESCUE_UAV_INDEX].telemetry.on_mission = True
                self.UAVs[RESCUE_UAV_INDEX].telemetry.mission_start_time = datetime.now().strftime("%Y%m%d_%H%M%S") # type: ignore
                if self.view.active_tab_index == RESCUE_UAV_INDEX:
                    self._set_pause_button_style("Pause")
                await asyncio.gather(
                    #uav_suspend_missions(drones=detected_uav_list, suspend_time=30),
                    uav_rescue_process(self.UAVs[RESCUE_UAV_INDEX], rescue_filepath, self)
                )
                self.UAVs[RESCUE_UAV_INDEX].telemetry.on_mission = False
                if self.view.active_tab_index == RESCUE_UAV_INDEX:
                    self._set_pause_button_style("Resume")
                self.UAVs[RESCUE_UAV_INDEX].rescue_first_time = False
                await asyncio.sleep(15)
                
                # 3 remove the rescue file
                if os.path.exists(rescue_filepath):
                    os.remove(rescue_filepath)  # remove the rescue file
                    self.logger.log(f"Rescue file {rescue_filepath} removed", level="info")
                
                break  # remove this line if you want to do the rescue mission continuously

            self.logger.log(f"Rescue mission completed", level="info")
        except Exception as e:
            self.logger.log(f"Error: {repr(e)}", level="error")
            self.view.popup_msg(f"Error: {repr(e)}", src_msg="uav_fn_rescue", type_msg="Error")

# ------------------------------------< Main Application Class >-----------------------------
def run():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Oxygen")  # ['Breeze', 'Oxygen', 'QtCurve', 'Windows', 'Fusion']
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    MainWindow = MainController()
    MainWindow.show()

    with loop:
        pending = asyncio.all_tasks(loop=loop)
        for task in pending:
            task.cancel()

        sys.exit(loop.run_forever())

if __name__ == "__main__":
    run()
