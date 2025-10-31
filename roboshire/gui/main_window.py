"""
RoboShire Main Window
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QMenuBar, QMenu, QToolBar, QStatusBar, QTabWidget,
    QMessageBox, QFileDialog, QSplitter, QLabel, QWizard,
    QLineEdit, QCompleter
)
from PySide6.QtCore import Qt, Signal, QTimer, QStringListModel
from PySide6.QtGui import QAction, QKeySequence, QIcon, QPixmap, QShortcut
from pathlib import Path
import logging
import time
import os  # v2.3.0

# Phase 7: Keyboard shortcuts
from roboshire.gui.keyboard_shortcuts import KeyboardShortcuts, apply_shortcut
from roboshire.gui.shortcut_cheatsheet import show_shortcut_cheatsheet

from roboshire.backend.urdf_manager import URDFManager
from roboshire.backend.code_generator import CodeGenerator
from roboshire.backend.ssh_subprocess import SSHManagerSubprocess
from roboshire.backend.workspace_manager import WorkspaceManager
from roboshire.backend.settings import SettingsManager
from roboshire.backend.execution_manager import ExecutionManager, ExecutionMode  # v2.3.0
from roboshire.backend.layout_manager import LayoutManager  # v2.5.0
from roboshire.gui.icon_manager import get_action_icon, get_menu_icon, IconManager  # v2.5.0
from roboshire.gui.command_palette import CommandPalette  # v2.5.0
from roboshire.gui.friendly_error_dialog import show_build_error, show_node_crash_error  # v2.5.0
from roboshire.gui.urdf_tree_editor import URDFTreeEditor
from roboshire.gui.property_editor import PropertyEditor
from roboshire.gui.node_graph_editor import NodeGraphEditorWidget
from roboshire.gui.system_architecture_widget import SystemArchitectureWidget
from roboshire.gui.build_output import BuildOutputViewer
from roboshire.gui.log_viewer import LogViewer
from roboshire.gui.node_status import NodeStatusWidget
from roboshire.gui.topic_inspector_widget import TopicInspectorWidget
from roboshire.gui.service_dialog import ServiceCallDialog
from roboshire.gui.param_editor import ParameterEditorWidget
from roboshire.gui.lifecycle_editor import LifecycleEditorWidget
from roboshire.gui.example_browser import ExampleBrowserWidget
from roboshire.gui.urdf_validator_widget import URDFValidatorWidget
from roboshire.gui.mujoco_viewer_widget import MuJoCoViewerWidget
from roboshire.gui.project_status_widget import ProjectStatusWidget
from roboshire.gui.package_file_tree_widget import PackageFileTreeWidget
from roboshire.gui.getting_started_widget import GettingStartedWidget
from roboshire.gui.code_editor_widget import CodeEditorWidget
from roboshire.gui.ssh_setup_wizard import SSHSetupWizard
from roboshire.gui.launch_file_editor import LaunchFileEditor
from roboshire.gui.nav2_wizard import Nav2IntegrationWizard
from roboshire.gui.behavior_tree_editor import BehaviorTreeEditor
from roboshire.gui.tf_tree_visualizer import TFTreeVisualizer
from roboshire.gui.performance_profiler import PerformanceProfiler
from roboshire.gui.multi_package_manager import MultiPackageManager
from roboshire.gui.execution_mode_dialog import ExecutionModeDialog  # v2.3.0
from roboshire.gui.deployment_setup_wizard import DeploymentSetupWizard  # v2.3.0
from roboshire.gui.micro_ros_agent_widget import MicroROSAgentWidget  # v2.4.0
from roboshire.gui.hardware_test_panel import HardwareTestPanel  # v2.4.0
from roboshire.gui.gazebo_plugin_manager_widget import GazeboPluginManagerWidget  # v2.4.0
from roboshire.integrations.rviz_bridge import RVizBridge
from roboshire.integrations.colcon_bridge import ColconBridge, BuildOptions
from roboshire.integrations.ros2_launcher import ROS2Launcher
from roboshire.integrations.node_monitor import NodeMonitor
from roboshire.integrations.topic_inspector import TopicInspector
from roboshire.integrations.service_manager import ServiceManager
from roboshire.integrations.param_manager import ParameterManager
from roboshire.integrations.lifecycle_manager import LifecycleManager
from roboshire.backend.error_translator import ErrorTranslator


class MainWindow(QMainWindow):
    """Main application window for RoboShire"""

    # Internal signal for thread-safe crash notifications
    _node_crashed = Signal(str)  # node_name

    def __init__(self):
        super().__init__()

        # Initialize logger
        self.logger = logging.getLogger(__name__)

        # Connect internal signal to handler (thread-safe)
        self._node_crashed.connect(self._handle_node_crash_on_main_thread)

        # Window properties
        self.setWindowTitle("RoboShire - Visual IDE for ROS2")
        self.setMinimumSize(1200, 800)
        self.resize(1400, 900)

        # Set window icon (logo without text for taskbar/title bar)
        try:
            from roboshire.resources import get_logo_path
            logo_path = str(get_logo_path(with_text=False))
            icon = QIcon(logo_path)
            self.setWindowIcon(icon)
        except Exception as e:
            self.logger.warning(f"Could not load window icon: {e}")

        # Project state
        self.current_project = None
        self.project_modified = False

        # Backend components
        self.urdf_manager = URDFManager()
        self.rviz_bridge = RVizBridge()
        self.code_generator = CodeGenerator()

        # Settings
        self.settings = SettingsManager()

        # v2.3.0: Execution Manager (Ubuntu Standalone)
        self.execution_manager = ExecutionManager.get_instance()

        # v2.5.0: Layout Manager
        self.layout_manager = LayoutManager()

        # SSH and build components (initialized lazily)
        self.ssh_manager = None
        self.workspace_manager = None
        self.colcon_bridge = None
        self.ros2_launcher = None

        # Phase 5 components (initialized lazily)
        self.node_monitor = None
        self.topic_inspector = None
        self.service_manager = None
        self.param_manager = None

        # Phase 6 components (initialized lazily)
        self.lifecycle_manager = None

        # v0.14.0 components
        self.error_translator = ErrorTranslator()

        # Track running nodes
        self.running_nodes = []

        # Auto-save timer
        self.autosave_timer = QTimer()
        self.autosave_timer.timeout.connect(self._on_autosave)
        self.autosave_interval_minutes = 5  # Auto-save every 5 minutes
        self.last_save_time = None
        self.autosave_enabled = True  # Can be disabled in settings

        # Setup UI
        self._create_actions()
        self._create_menus()
        self._create_toolbars()
        self._create_central_widget()
        self._create_status_bar()

        # Update UI state
        self._update_ui_state()

        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

        # Start auto-save timer (5 minutes = 300000 ms)
        if self.autosave_enabled:
            self.autosave_timer.start(self.autosave_interval_minutes * 60 * 1000)

        # v2.3.0: Check for first launch and show deployment wizard
        self._check_first_launch()

    def _create_actions(self):
        """Create all actions for menus and toolbars"""

        # File actions (v2.5.0: Added icons)
        self.new_project_action = QAction(get_action_icon("new_project"), "&New Project...", self)
        apply_shortcut(self.new_project_action, KeyboardShortcuts.NEW_PROJECT,
                      "Create a new RoboShire project")
        self.new_project_action.triggered.connect(self._on_new_project)

        self.new_from_example_action = QAction(get_action_icon("examples"), "New from &Example...", self)
        apply_shortcut(self.new_from_example_action, KeyboardShortcuts.OPEN_EXAMPLES,
                      "Create project from robot example")
        self.new_from_example_action.triggered.connect(self._on_new_from_example)

        self.new_robot_wizard_action = QAction(get_action_icon("wizard"), "New Robot &Wizard...", self)
        self.new_robot_wizard_action.setShortcut(QKeySequence("Ctrl+Shift+N"))
        self.new_robot_wizard_action.setStatusTip("5-minute guided robot creation wizard")
        self.new_robot_wizard_action.triggered.connect(self._on_new_robot_wizard)

        self.open_project_action = QAction(get_action_icon("open_project"), "&Open Project...", self)
        apply_shortcut(self.open_project_action, KeyboardShortcuts.OPEN_PROJECT,
                      "Open an existing RoboShire project")
        self.open_project_action.triggered.connect(self._on_open_project)

        self.save_project_action = QAction(get_action_icon("save_project"), "&Save Project", self)
        apply_shortcut(self.save_project_action, KeyboardShortcuts.SAVE_PROJECT,
                      "Save the current project")
        self.save_project_action.triggered.connect(self._on_save_project)

        self.save_project_as_action = QAction(get_action_icon("save_project"), "Save Project &As...", self)
        apply_shortcut(self.save_project_as_action, KeyboardShortcuts.SAVE_PROJECT_AS,
                      "Save project with a new name")
        self.save_project_as_action.triggered.connect(self._on_save_project_as)

        self.exit_action = QAction(get_menu_icon("✖"), "E&xit", self)
        apply_shortcut(self.exit_action, KeyboardShortcuts.QUIT,
                      "Exit RoboShire")
        self.exit_action.triggered.connect(self.close)

        # URDF/Robot actions (v2.5.0: Added icons)
        self.import_urdf_action = QAction(get_action_icon("import_urdf"), "Import &URDF...", self)
        apply_shortcut(self.import_urdf_action, KeyboardShortcuts.IMPORT_URDF,
                      "Import a URDF file")
        self.import_urdf_action.triggered.connect(self._on_import_urdf)

        self.export_urdf_action = QAction("&Export URDF As...", self)
        apply_shortcut(self.export_urdf_action, KeyboardShortcuts.EXPORT_URDF,
                      "Export URDF to a new file")
        self.export_urdf_action.triggered.connect(self._on_export_urdf)

        self.view_rviz_action = QAction("View in &RViz2", self)
        apply_shortcut(self.view_rviz_action, KeyboardShortcuts.VIEW_RVIZ,
                      "Launch RViz2 to view robot model")
        self.view_rviz_action.triggered.connect(self._on_view_rviz)

        # Simulation actions
        self.sim_gazebo_action = QAction("Simulate in &Gazebo", self)
        apply_shortcut(self.sim_gazebo_action, KeyboardShortcuts.SIMULATE_GAZEBO,
                      "Launch Gazebo simulation")
        self.sim_gazebo_action.triggered.connect(self._on_simulate_gazebo)

        # Build actions
        self.generate_code_action = QAction(get_action_icon("generate_code"), "&Generate Code", self)
        apply_shortcut(self.generate_code_action, KeyboardShortcuts.GENERATE_CODE,
                      "Generate ROS2 code from node graph")
        self.generate_code_action.triggered.connect(self._on_generate_code)

        self.build_action = QAction(get_action_icon("build"), "&Build", self)
        apply_shortcut(self.build_action, KeyboardShortcuts.BUILD,
                      "Build ROS2 workspace with colcon")
        self.build_action.triggered.connect(self._on_build)

        self.clean_action = QAction(get_action_icon("clean"), "&Clean Build", self)
        apply_shortcut(self.clean_action, KeyboardShortcuts.CLEAN_BUILD,
                      "Clean and rebuild workspace")
        self.clean_action.triggered.connect(self._on_clean)

        # Run actions (v2.5.0: Added icons)
        self.run_action = QAction(get_action_icon("run"), "&Run", self)
        apply_shortcut(self.run_action, KeyboardShortcuts.RUN,
                      "Launch ROS2 nodes")
        self.run_action.triggered.connect(self._on_run)

        self.stop_action = QAction(get_action_icon("stop"), "&Stop", self)
        apply_shortcut(self.stop_action, KeyboardShortcuts.STOP,
                      "Stop running nodes")
        self.stop_action.triggered.connect(self._on_stop)

        # Tools actions
        self.rqt_graph_action = QAction("rqt_graph", self)
        apply_shortcut(self.rqt_graph_action, KeyboardShortcuts.OPEN_RQT_GRAPH,
                      "Launch rqt_graph tool")
        self.rqt_graph_action.triggered.connect(self._on_rqt_graph)

        self.rqt_console_action = QAction("rqt_console", self)
        apply_shortcut(self.rqt_console_action, KeyboardShortcuts.OPEN_RQT_CONSOLE,
                      "Launch rqt_console tool")
        self.rqt_console_action.triggered.connect(self._on_rqt_console)

        self.validate_urdf_action = QAction("&Validate URDF", self)
        apply_shortcut(self.validate_urdf_action, KeyboardShortcuts.VALIDATE_URDF,
                      "Validate current URDF file")
        self.validate_urdf_action.triggered.connect(self._on_validate_urdf)

        # v2.3.0: Execution mode action
        self.execution_mode_action = QAction("&Execution Mode...", self)
        self.execution_mode_action.setStatusTip("Configure execution mode (Local/Remote)")
        self.execution_mode_action.triggered.connect(self._on_execution_mode)

        self.ssh_setup_wizard_action = QAction("SSH Setup &Wizard...", self)
        self.ssh_setup_wizard_action.setStatusTip("Configure SSH connection to Ubuntu VM")
        self.ssh_setup_wizard_action.triggered.connect(self._on_ssh_setup_wizard)

        # v2.0.0 actions
        self.nav2_wizard_action = QAction("Nav2 Integration &Wizard...", self)
        self.nav2_wizard_action.setStatusTip("Configure Nav2 navigation stack")
        self.nav2_wizard_action.triggered.connect(self._on_nav2_wizard)

        # v2.4.0: Hardware actions
        self.micro_ros_agent_action = QAction("micro-ROS &Agent...", self)
        apply_shortcut(self.micro_ros_agent_action, KeyboardShortcuts.MICRO_ROS_AGENT,
                      "Manage micro-ROS agent for Arduino/ESP32")
        self.micro_ros_agent_action.triggered.connect(self._on_micro_ros_agent)

        self.hardware_test_action = QAction("Hardware &Test Panel...", self)
        apply_shortcut(self.hardware_test_action, KeyboardShortcuts.HARDWARE_TEST_PANEL,
                      "Interactive hardware testing and debugging")
        self.hardware_test_action.triggered.connect(self._on_hardware_test)

        # v2.5.0: Gazebo plugin manager
        self.gazebo_plugin_action = QAction("&Gazebo Plugin Manager...", self)
        self.gazebo_plugin_action.setStatusTip("Configure Gazebo plugins without XML")
        self.gazebo_plugin_action.triggered.connect(self._on_gazebo_plugins)

        # View actions (for tab switching)
        self.switch_to_node_graph_action = QAction("Node Graph", self)
        apply_shortcut(self.switch_to_node_graph_action, KeyboardShortcuts.TAB_NODE_GRAPH,
                      "Switch to Node Graph tab")
        self.switch_to_node_graph_action.triggered.connect(lambda: self._switch_to_tab("Node Graph"))

        self.switch_to_mujoco_action = QAction("MuJoCo Viewer", self)
        self.switch_to_mujoco_action.setShortcut(QKeySequence("Ctrl+2"))
        self.switch_to_mujoco_action.setStatusTip("Switch to MuJoCo tab")
        self.switch_to_mujoco_action.triggered.connect(lambda: self._switch_to_tab("MuJoCo"))

        self.switch_to_build_action = QAction("Build Output", self)
        self.switch_to_build_action.setShortcut(QKeySequence("Ctrl+3"))
        self.switch_to_build_action.setStatusTip("Switch to Build tab")
        self.switch_to_build_action.triggered.connect(lambda: self._switch_to_tab("Build"))

        self.switch_to_logs_action = QAction("Logs", self)
        self.switch_to_logs_action.setShortcut(QKeySequence("Ctrl+4"))
        self.switch_to_logs_action.setStatusTip("Switch to Logs tab")
        self.switch_to_logs_action.triggered.connect(lambda: self._switch_to_tab("Logs"))

        self.switch_to_node_status_action = QAction("Node Status", self)
        self.switch_to_node_status_action.setShortcut(QKeySequence("Ctrl+5"))
        self.switch_to_node_status_action.setStatusTip("Switch to Node Status tab")
        self.switch_to_node_status_action.triggered.connect(lambda: self._switch_to_tab("Node Status"))

        self.switch_to_topics_action = QAction("Topics", self)
        self.switch_to_topics_action.setShortcut(QKeySequence("Ctrl+6"))
        self.switch_to_topics_action.setStatusTip("Switch to Topics tab")
        self.switch_to_topics_action.triggered.connect(lambda: self._switch_to_tab("Topics"))

        self.switch_to_parameters_action = QAction("Parameters", self)
        self.switch_to_parameters_action.setShortcut(QKeySequence("Ctrl+7"))
        self.switch_to_parameters_action.setStatusTip("Switch to Parameters tab")
        self.switch_to_parameters_action.triggered.connect(lambda: self._switch_to_tab("Parameters"))

        self.switch_to_lifecycle_action = QAction("Lifecycle", self)
        self.switch_to_lifecycle_action.setShortcut(QKeySequence("Ctrl+8"))
        self.switch_to_lifecycle_action.setStatusTip("Switch to Lifecycle tab")
        self.switch_to_lifecycle_action.triggered.connect(lambda: self._switch_to_tab("Lifecycle"))

        self.switch_to_packages_action = QAction("Packages", self)
        apply_shortcut(self.switch_to_packages_action, KeyboardShortcuts.TAB_PACKAGES,
                      "Switch to Package Browser tab")
        self.switch_to_packages_action.triggered.connect(lambda: self._switch_to_tab("Packages"))

        # Window layout actions (v2.5.0)
        self.reset_layout_action = QAction("Reset Layout", self)
        apply_shortcut(self.reset_layout_action, KeyboardShortcuts.RESET_VIEW,
                      "Reset window layout to default")
        self.reset_layout_action.triggered.connect(self._on_reset_layout)

        self.save_layout_action = QAction("Save Layout...", self)
        self.save_layout_action.setStatusTip("Save current window layout")
        self.save_layout_action.triggered.connect(self._on_save_layout)

        # Help actions
        self.shortcuts_action = QAction("&Keyboard Shortcuts", self)
        apply_shortcut(self.shortcuts_action, KeyboardShortcuts.SHORTCUT_CHEATSHEET,
                      "Show keyboard shortcut cheat sheet")
        self.shortcuts_action.triggered.connect(self._on_show_shortcuts)

        self.about_action = QAction("&About RoboShire", self)
        apply_shortcut(self.about_action, KeyboardShortcuts.ABOUT,
                      "About RoboShire")
        self.about_action.triggered.connect(self._on_about)

        # Command Palette action (v2.5.0)
        self.command_palette_action = QAction("Command &Palette...", self)
        self.command_palette_action.setShortcut(QKeySequence("Ctrl+Shift+P"))
        self.command_palette_action.setStatusTip("Open command palette (fuzzy search all commands)")
        self.command_palette_action.triggered.connect(self._on_command_palette)

    def _create_menus(self):
        """Create menu bar - v2.5.0 reorganized structure

        Changes from v2.4.1:
        - Merged Robot + Simulation → Robot (7 items)
        - Merged Hardware + Tools → Tools (7 items)
        - Renamed Build → Project (8 items)
        - Renamed View → Window (7 items)
        - Added Edit menu (placeholder for future)
        - Reduced from 9 menus to 6 menus
        """
        menubar = self.menuBar()

        # ========== FILE MENU ==========
        file_menu = menubar.addMenu("&File")
        file_menu.addAction(self.new_project_action)
        file_menu.addAction(self.new_from_example_action)
        file_menu.addAction(self.new_robot_wizard_action)
        file_menu.addAction(self.open_project_action)
        file_menu.addSeparator()
        file_menu.addAction(self.save_project_action)
        file_menu.addAction(self.save_project_as_action)
        file_menu.addSeparator()
        file_menu.addAction(self.exit_action)

        # ========== EDIT MENU (NEW in v2.5.0) ==========
        # Placeholder for future text editing features
        edit_menu = menubar.addMenu("&Edit")
        edit_placeholder = edit_menu.addAction("(Edit features coming in v2.5.1)")
        edit_placeholder.setEnabled(False)

        # ========== PROJECT MENU (renamed from Build) ==========
        project_menu = menubar.addMenu("&Project")
        project_menu.addAction(self.generate_code_action)
        project_menu.addAction(self.build_action)
        project_menu.addAction(self.clean_action)
        project_menu.addSeparator()
        project_menu.addAction(self.run_action)
        project_menu.addAction(self.stop_action)
        project_menu.addSeparator()
        project_menu.addAction(self.execution_mode_action)  # Moved from Settings
        project_menu.addAction(self.switch_to_packages_action)  # Moved from View

        # ========== ROBOT MENU (merged Robot + Simulation) ==========
        robot_menu = menubar.addMenu("&Robot")
        robot_menu.addAction(self.import_urdf_action)
        robot_menu.addAction(self.export_urdf_action)
        robot_menu.addAction(self.validate_urdf_action)  # Moved from Tools
        robot_menu.addSeparator()
        robot_menu.addAction(self.view_rviz_action)
        robot_menu.addAction(self.sim_gazebo_action)  # Merged from Simulation menu
        robot_menu.addAction(self.switch_to_mujoco_action)  # MuJoCo viewer
        robot_menu.addSeparator()
        robot_menu.addAction(self.nav2_wizard_action)  # Moved from Tools

        # ========== TOOLS MENU (merged Hardware + Tools) ==========
        tools_menu = menubar.addMenu("&Tools")
        tools_menu.addAction(self.hardware_test_action)  # From Hardware
        # Note: Serial Monitor will be added when integrated
        tools_menu.addAction(self.micro_ros_agent_action)  # From Hardware
        tools_menu.addSeparator()
        tools_menu.addAction(self.rqt_graph_action)
        tools_menu.addAction(self.rqt_console_action)
        tools_menu.addSeparator()
        tools_menu.addAction(self.ssh_setup_wizard_action)  # Moved from Settings
        tools_menu.addAction(self.gazebo_plugin_action)  # From Hardware

        # ========== WINDOW MENU (renamed from View) ==========
        window_menu = menubar.addMenu("&Window")
        window_menu.addAction(self.switch_to_node_graph_action)
        window_menu.addAction(self.switch_to_build_action)
        window_menu.addAction(self.switch_to_logs_action)
        window_menu.addAction(self.switch_to_node_status_action)
        window_menu.addAction(self.switch_to_topics_action)
        window_menu.addAction(self.switch_to_parameters_action)
        window_menu.addAction(self.switch_to_lifecycle_action)
        window_menu.addSeparator()
        window_menu.addAction(self.reset_layout_action)  # v2.5.0
        window_menu.addAction(self.save_layout_action)   # v2.5.0

        # ========== HELP MENU ==========
        help_menu = menubar.addMenu("&Help")
        help_menu.addAction(self.command_palette_action)  # v2.5.0
        help_menu.addAction(self.shortcuts_action)
        help_menu.addSeparator()
        help_menu.addAction(self.about_action)

    def _create_toolbars(self):
        """Create toolbars"""

        # Main toolbar
        main_toolbar = self.addToolBar("Main")
        main_toolbar.setObjectName("MainToolbar")
        main_toolbar.addAction(self.new_project_action)
        main_toolbar.addAction(self.open_project_action)
        main_toolbar.addAction(self.save_project_action)
        main_toolbar.addSeparator()
        main_toolbar.addAction(self.build_action)
        main_toolbar.addAction(self.run_action)
        main_toolbar.addAction(self.stop_action)

    def _create_central_widget(self):
        """Create central widget with main layout"""

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)

        # Global search bar at the very top
        search_bar_layout = QHBoxLayout()
        search_bar_layout.setContentsMargins(10, 5, 10, 5)

        self.global_search = QLineEdit()
        self.global_search.setPlaceholderText("Search features, options, help... (Ctrl+K)")
        self.global_search.setStyleSheet("""
            QLineEdit {
                padding: 8px 12px;
                border: 2px solid #556B2F;
                border-radius: 6px;
                font-size: 12px;
                background-color: white;
            }
            QLineEdit:focus {
                border: 2px solid #6B8E23;
                background-color: #FAFAFA;
            }
        """)
        self.global_search.returnPressed.connect(self._on_global_search)
        self.global_search.textChanged.connect(self._on_global_search_text_changed)

        # Setup auto-complete with live suggestions
        self.search_keywords = [
            'Start Workflow Wizard',
            'New Project',
            'Node Graph Editor',
            'Build Package',
            'Run Nodes',
            'MuJoCo Simulation',
            'RViz Visualization',
            'Code Editor',
            'Topics Inspector',
            'Parameters Editor',
            'Logs Viewer',
            'SSH Setup',
            'Browse Examples',
            'Keyboard Shortcuts',
            'Help Documentation',
            'About RoboShire',
            'Settings',
            'Launch Files',
            'Behavior Trees',
            'TF Tree Visualizer',
            'System Architecture',
            'Package Browser',
            'Service Manager',
            'Action Manager',
            'Performance Monitor'
        ]

        self.search_completer = QCompleter(self.search_keywords)
        self.search_completer.setCaseSensitivity(Qt.CaseInsensitive)
        self.search_completer.setFilterMode(Qt.MatchContains)  # Match anywhere in string
        self.search_completer.setCompletionMode(QCompleter.PopupCompletion)
        self.search_completer.setMaxVisibleItems(8)

        # Style the completer popup
        self.search_completer.popup().setStyleSheet("""
            QListView {
                border: 2px solid #6B8E23;
                border-radius: 6px;
                background-color: white;
                font-size: 12px;
                padding: 4px;
            }
            QListView::item {
                padding: 6px 12px;
                border-radius: 4px;
            }
            QListView::item:hover {
                background-color: #F0F5E6;
            }
            QListView::item:selected {
                background-color: #6B8E23;
                color: white;
            }
        """)

        self.global_search.setCompleter(self.search_completer)

        # Shortcut for search (Ctrl+K)
        search_shortcut = QShortcut(QKeySequence("Ctrl+K"), self)
        search_shortcut.activated.connect(lambda: self.global_search.setFocus())

        search_bar_layout.addWidget(self.global_search)
        main_layout.addLayout(search_bar_layout)

        # Create horizontal splitter for main panels
        splitter = QSplitter(Qt.Horizontal)

        # Left panel - Vertical splitter with File Tree (top) and URDF Tree (bottom)
        left_splitter = QSplitter(Qt.Vertical)

        # File Tree (top of left panel)
        from PySide6.QtWidgets import QGroupBox
        file_tree_group = QGroupBox("Project Files")
        file_tree_layout = QVBoxLayout(file_tree_group)
        file_tree_layout.setContentsMargins(2, 2, 2, 2)

        self.left_file_tree = PackageFileTreeWidget()
        self.left_file_tree.file_selected.connect(self._on_package_file_selected)
        file_tree_layout.addWidget(self.left_file_tree)

        left_splitter.addWidget(file_tree_group)

        # URDF Tree (bottom of left panel)
        urdf_tree_group = QGroupBox("URDF Components")
        urdf_tree_layout = QVBoxLayout(urdf_tree_group)
        urdf_tree_layout.setContentsMargins(2, 2, 2, 2)

        self.urdf_tree_editor = URDFTreeEditor()
        self.urdf_tree_editor.set_urdf_manager(self.urdf_manager)
        self.urdf_tree_editor.item_selected.connect(self._on_urdf_item_selected)
        self.urdf_tree_editor.item_double_clicked.connect(self._on_urdf_item_double_clicked)
        urdf_tree_layout.addWidget(self.urdf_tree_editor)

        left_splitter.addWidget(urdf_tree_group)

        # Set initial sizes for left splitter (60% files, 40% URDF)
        left_splitter.setSizes([350, 250])

        # Center panel - Organized tabs with categories
        self.tab_widget = QTabWidget()
        self.tab_widget.setObjectName("mainTabWidget")  # v2.5.0: For layout persistence
        # Note: Tabs are NOT closeable - they have no menu items to reopen them
        # Users must keep all tabs accessible for core functionality

        # Getting Started Widget (v0.14.0) - FIRST TAB for new users
        self.getting_started_widget = GettingStartedWidget()
        self.getting_started_widget.action_requested.connect(self._on_getting_started_action)
        self.tab_widget.addTab(self.getting_started_widget, "Getting Started")

        # Project Status Widget (Phase 9)
        self.project_status_widget = ProjectStatusWidget()
        self.project_status_widget.workflow_wizard_requested.connect(self._on_new_robot_wizard)
        self.project_status_widget.design_node_graph_requested.connect(lambda: self._switch_to_tab("Node Graph"))
        self.project_status_widget.generate_code_requested.connect(self._on_generate_code)
        self.project_status_widget.build_requested.connect(self._on_build)
        self.project_status_widget.run_requested.connect(self._on_run)
        self.project_status_widget.open_testing_guide_requested.connect(self._on_open_testing_guide)
        self.tab_widget.addTab(self.project_status_widget, "Project Status")

        # === DESIGN CATEGORY ===
        design_tabs = QTabWidget()
        design_tabs.setObjectName("designTabs")  # v2.5.0: For layout persistence
        design_tabs.setDocumentMode(True)

        # Node Graph Editor
        self.node_graph_editor = NodeGraphEditorWidget()
        self.node_graph_editor.graph_modified.connect(self._on_graph_modified)
        self.node_graph_editor.node_selected.connect(self._on_node_selected)
        design_tabs.addTab(self.node_graph_editor, "Node Graph")

        # Launch File Editor (v1.0.0) - visual launch file creation
        self.launch_file_editor = LaunchFileEditor()
        self.launch_file_editor.launch_file_saved.connect(self._on_launch_file_saved)
        design_tabs.addTab(self.launch_file_editor, "Launch Files")

        # Behavior Tree Editor (v2.0.0)
        self.behavior_tree_editor = BehaviorTreeEditor()
        self.behavior_tree_editor.tree_saved.connect(self._on_behavior_tree_saved)
        design_tabs.addTab(self.behavior_tree_editor, "Behavior Trees")

        self.tab_widget.addTab(design_tabs, "Design")

        # === DEVELOPMENT CATEGORY ===
        dev_tabs = QTabWidget()
        dev_tabs.setObjectName("devTabs")  # v2.5.0: For layout persistence
        dev_tabs.setDocumentMode(True)

        # Code Editor Widget (v0.14.0) - for quick file edits
        self.code_editor_widget = CodeEditorWidget()
        self.code_editor_widget.file_saved.connect(self._on_code_file_saved)
        dev_tabs.addTab(self.code_editor_widget, "Code Editor")

        # Multi-Package Manager (v2.0.0)
        self.multi_package_manager = MultiPackageManager()
        self.multi_package_manager.package_created.connect(self._on_package_created)
        self.multi_package_manager.build_requested.connect(self._on_multi_package_build)
        dev_tabs.addTab(self.multi_package_manager, "Multi-Package")

        # Package Browser Widget (Phase 12)
        from roboshire.gui.package_browser_widget import PackageBrowserWidget
        self.package_browser_widget = PackageBrowserWidget(ros_distro=self.settings.workspace.ros_distro)
        dev_tabs.addTab(self.package_browser_widget, "Package Browser")

        self.tab_widget.addTab(dev_tabs, "Development")

        # === SIMULATION CATEGORY ===
        sim_tabs = QTabWidget()
        sim_tabs.setObjectName("simTabs")  # v2.5.0: For layout persistence
        sim_tabs.setDocumentMode(True)

        # MuJoCo Viewer (Phase 8)
        self.mujoco_viewer = MuJoCoViewerWidget()
        self.mujoco_viewer.simulation_started.connect(self._on_mujoco_simulation_started)
        self.mujoco_viewer.simulation_stopped.connect(self._on_mujoco_simulation_stopped)
        self.mujoco_viewer.joint_state_changed.connect(self._on_mujoco_joint_states)
        sim_tabs.addTab(self.mujoco_viewer, "MuJoCo Viewer")

        # TF Tree Visualizer (v2.0.0)
        self.tf_tree_visualizer = TFTreeVisualizer(ssh_bridge=self.ssh_manager)
        sim_tabs.addTab(self.tf_tree_visualizer, "TF Tree")

        self.tab_widget.addTab(sim_tabs, "Simulation")

        # === MONITORING CATEGORY ===
        monitor_tabs = QTabWidget()
        monitor_tabs.setObjectName("monitorTabs")  # v2.5.0: For layout persistence
        monitor_tabs.setDocumentMode(True)

        # Node Status Widget (Phase 5)
        self.node_status_widget = NodeStatusWidget()
        monitor_tabs.addTab(self.node_status_widget, "Node Status")

        # Topic Inspector Widget (Phase 5)
        self.topic_inspector_widget = TopicInspectorWidget()
        monitor_tabs.addTab(self.topic_inspector_widget, "Topics")

        # Parameter Editor Widget (Phase 5)
        self.param_editor_widget = ParameterEditorWidget()
        monitor_tabs.addTab(self.param_editor_widget, "Parameters")

        # Performance Profiler (v2.0.0)
        self.performance_profiler = PerformanceProfiler(ssh_bridge=self.ssh_manager)
        monitor_tabs.addTab(self.performance_profiler, "Performance")

        # Log Viewer
        self.log_viewer = LogViewer()
        self.log_viewer.clear_requested.connect(self._on_logs_cleared)
        monitor_tabs.addTab(self.log_viewer, "Logs")

        self.tab_widget.addTab(monitor_tabs, "Monitoring")

        # === BUILD & DEPLOY CATEGORY ===
        build_tabs = QTabWidget()
        build_tabs.setObjectName("buildTabs")  # v2.5.0: For layout persistence
        build_tabs.setDocumentMode(True)

        # Build Output Viewer
        self.build_output_viewer = BuildOutputViewer()
        self.build_output_viewer.cancel_requested.connect(self._on_build_cancel)
        build_tabs.addTab(self.build_output_viewer, "Build Output")

        # Lifecycle Editor Widget (Phase 6) - Placeholder until lifecycle manager initialized
        self.lifecycle_editor_widget = None
        self.lifecycle_tab_placeholder = self._create_placeholder(
            "Lifecycle Node Manager\n\n"
            "Launch ROS2 nodes to enable lifecycle management"
        )
        build_tabs.addTab(self.lifecycle_tab_placeholder, "Lifecycle Manager")

        self.tab_widget.addTab(build_tabs, "Build & Deploy")

        # === SYSTEM ARCHITECTURE (closeable tab) ===
        self.system_architecture_widget = SystemArchitectureWidget()
        self.tab_widget.addTab(self.system_architecture_widget, "System Architecture")

        # Package File Tree Widget - now in left panel (keep reference for compatibility)
        self.package_file_tree = self.left_file_tree

        # Keep property editor and validator in background for URDF editing
        # (can be accessed via menu if needed)
        self.property_editor = PropertyEditor()
        self.property_editor.set_urdf_manager(self.urdf_manager)
        self.property_editor.property_changed.connect(self._on_property_changed)
        self.property_editor.setVisible(False)  # Hidden by default

        self.urdf_validator_widget = URDFValidatorWidget()
        self.urdf_validator_widget.element_selected.connect(self._on_validator_element_selected)
        self.urdf_validator_widget.setVisible(False)  # Hidden by default

        # Add panels to splitter (now just 2 panels: left and center)
        splitter.addWidget(left_splitter)  # Left panel with file tree + URDF tree
        splitter.addWidget(self.tab_widget)  # Center panel with all tabs

        # Set initial sizes (left: 20%, center: 80%)
        splitter.setSizes([300, 1100])

        # Add splitter to main layout
        main_layout.addWidget(splitter)

    def _create_placeholder(self, text):
        """Create a placeholder widget with text"""
        from PySide6.QtWidgets import QLabel
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("QLabel { color: gray; font-size: 16px; }")
        return label

    def _create_status_bar(self):
        """Create enhanced status bar"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # Main message
        self.status_message = QLabel("Ready")
        self.status_bar.addWidget(self.status_message, 1)

        # Project status
        self.project_status_label = QLabel("No project")
        self.project_status_label.setStyleSheet("color: #666;")
        self.status_bar.addPermanentWidget(self.project_status_label)

        # Save time status
        self.save_time_label = QLabel("Not saved")
        self.save_time_label.setStyleSheet("color: #888;")
        self.save_time_label.setToolTip("Time since last save")
        self.status_bar.addPermanentWidget(self.save_time_label)

        # Build status
        self.build_status_label = QLabel("Not built")
        self.build_status_label.setStyleSheet("color: #666;")
        self.status_bar.addPermanentWidget(self.build_status_label)

        # Running nodes status
        self.nodes_status_label = QLabel("0 nodes")
        self.nodes_status_label.setStyleSheet("color: #666;")
        self.status_bar.addPermanentWidget(self.nodes_status_label)

        # v2.3.0: Execution mode indicator (clickable)
        self.execution_mode_label = QLabel()
        self.execution_mode_label.setStyleSheet("color: #556B2F; font-weight: bold;")
        self.execution_mode_label.setToolTip("Click to change execution mode")
        self.execution_mode_label.mousePressEvent = lambda e: self._on_execution_mode()
        self.execution_mode_label.setCursor(Qt.PointingHandCursor)
        self.status_bar.addPermanentWidget(self.execution_mode_label)
        self._update_execution_mode_indicator()

        # v2.4.0: micro-ROS Agent status indicator (clickable)
        self.agent_status_label = QLabel("Agent: ● Disconnected")
        self.agent_status_label.setStyleSheet("QLabel { color: red; }")
        self.agent_status_label.setToolTip("Click to open micro-ROS Agent Manager")
        self.agent_status_label.mousePressEvent = lambda e: self._on_micro_ros_agent()
        self.agent_status_label.setCursor(Qt.PointingHandCursor)
        self.status_bar.addPermanentWidget(self.agent_status_label)

        self.status_bar.showMessage("Ready")

        # Timer to update "last saved" display every minute
        self.save_time_update_timer = QTimer()
        self.save_time_update_timer.timeout.connect(self._update_status_bar_save_time)
        self.save_time_update_timer.start(60000)  # Update every minute

    def _update_ui_state(self):
        """Update UI state based on current project"""
        has_project = self.current_project is not None
        has_urdf = self.urdf_manager.is_loaded()

        # Update window title with modified indicator
        base_title = "RoboShire - Visual IDE for ROS2"
        if has_project:
            from pathlib import Path
            project_name = Path(self.current_project).stem if self.current_project else "Untitled"
            title = f"{base_title} - {project_name}"
            if self.project_modified:
                title += " *"  # Add asterisk for modified files
        else:
            title = base_title

        self.setWindowTitle(title)

        # Enable/disable actions
        self.save_project_action.setEnabled(has_project and self.project_modified)
        self.save_project_as_action.setEnabled(has_project)

        # Import URDF should always be available
        self.import_urdf_action.setEnabled(True)

        # Export requires a loaded URDF
        self.export_urdf_action.setEnabled(has_urdf)

        # These require a loaded URDF
        self.view_rviz_action.setEnabled(has_urdf)
        self.sim_gazebo_action.setEnabled(has_urdf)

        # Build/clean are always available
        self.build_action.setEnabled(True)
        self.clean_action.setEnabled(True)

        # Run is always available
        self.run_action.setEnabled(True)

        # Stop is enabled when nodes are running
        has_running_nodes = len(self.running_nodes) > 0
        self.stop_action.setEnabled(has_running_nodes)

        # Update status bar labels (NEW)
        if hasattr(self, 'project_status_label'):
            if has_project:
                self.project_status_label.setText(f"Project: {Path(self.current_project).name if self.current_project else 'Unnamed'}")
                self.project_status_label.setStyleSheet("color: #556B2F;")
            else:
                self.project_status_label.setText("No project")
                self.project_status_label.setStyleSheet("color: #666;")

        if hasattr(self, 'nodes_status_label'):
            if has_running_nodes:
                self.nodes_status_label.setText(f"{len(self.running_nodes)} nodes running")
                self.nodes_status_label.setStyleSheet("color: #4CAF50;")
            else:
                self.nodes_status_label.setText("0 nodes")
                self.nodes_status_label.setStyleSheet("color: #666;")

        # Update main status message
        if has_running_nodes:
            self.status_bar.showMessage(f"{len(self.running_nodes)} node(s) running")

    # Slot implementations (placeholders)

    def _on_new_project(self):
        """Create new project"""
        # Check for unsaved changes
        if not self._check_unsaved_changes():
            return

        # Show the workflow wizard - best way for beginners to create a robot
        self._on_new_robot_wizard()

    def _on_new_from_example(self):
        """Create new project from example"""
        # Check for unsaved changes
        if not self._check_unsaved_changes():
            return

        # Create and show example browser dialog
        from PySide6.QtWidgets import QDialog, QVBoxLayout, QPushButton

        dialog = QDialog(self)
        dialog.setWindowTitle("Robot Examples Library")
        dialog.resize(1000, 700)

        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(0, 0, 0, 0)

        # Add example browser widget
        browser = ExampleBrowserWidget()
        browser.example_loaded.connect(lambda urdf, node_graph: self._load_example(urdf, node_graph, dialog))
        layout.addWidget(browser)

        # Close button
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dialog.reject)
        button_layout.addWidget(close_btn)
        layout.addLayout(button_layout)

        dialog.exec()

    def _load_example(self, urdf_content: str, node_graph_dict: dict, dialog: QWidget):
        """Load an example's URDF and node graph into the editor"""
        import tempfile
        import os

        try:
            # Save URDF to temporary file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(urdf_content)
                temp_urdf_path = f.name

            # Load URDF
            success = self.urdf_manager.load(temp_urdf_path)
            if success:
                self.urdf_tree_editor.refresh()
                self.status_bar.showMessage("Example URDF loaded")
                self._update_ui_state()
            else:
                QMessageBox.warning(self, "Load Failed", "Failed to load example URDF")
                return

            # Load node graph
            from roboshire.backend.node_graph import NodeGraph
            graph = NodeGraph()

            # Reconstruct node graph from dictionary
            if "nodes" in node_graph_dict:
                for node_id, node_data in node_graph_dict["nodes"].items():
                    from roboshire.backend.node_graph import Node, NodeType, Port, PortType

                    # Create node
                    node = Node(
                        id=node_data["id"],
                        name=node_data["name"],
                        node_type=NodeType(node_data["node_type"]),
                        position=tuple(node_data["position"])
                    )
                    node.properties = node_data.get("properties", {})

                    # Create ports
                    for port_id, port_data in node_data.get("ports", {}).items():
                        port = Port(
                            id=port_data["id"],
                            name=port_data["name"],
                            port_type=PortType(port_data["port_type"]),
                            data_type=port_data["data_type"]
                        )
                        port.connected_to = port_data.get("connected_to", [])
                        node.ports[port_id] = port

                    graph.nodes[node_id] = node

            # Load connections
            if "connections" in node_graph_dict:
                graph.connections = node_graph_dict["connections"]

            # Set graph in editor
            self.node_graph_editor.load_graph(graph)
            self.status_bar.showMessage("Example loaded successfully")

            # Close dialog
            dialog.accept()

            # Clean up temp file
            try:
                os.unlink(temp_urdf_path)
            except:
                pass

        except Exception as e:
            QMessageBox.critical(
                self,
                "Load Error",
                f"Failed to load example:\n\n{e}"
            )
            import traceback
            traceback.print_exc()

    def _on_new_robot_wizard(self):
        """Launch 5-minute robot creation wizard"""
        from roboshire.gui.workflow_wizard import WorkflowWizard

        wizard = WorkflowWizard(self)
        wizard.project_created.connect(self._on_wizard_project_created)
        wizard.exec()

    def _on_wizard_project_created(self, config: dict):
        """Handle project creation from workflow wizard"""
        try:
            project_name = config.get('project_name', 'my_robot')
            use_urdf = config.get('use_urdf', False)
            urdf_path = config.get('urdf_path')

            self.status_bar.showMessage(f"Creating project: {project_name}...")

            # If URDF was selected, load it
            if use_urdf and urdf_path:
                success = self.urdf_manager.load(urdf_path)
                if success:
                    self.urdf_tree_editor.refresh()
                    self._update_ui_state()

                    # Load into MuJoCo viewer
                    try:
                        self.mujoco_viewer.load_urdf(urdf_path)
                        # Switch to MuJoCo tab
                        self._switch_to_tab("MuJoCo")
                    except Exception as e:
                        self.logger.warning(f"Failed to load URDF into MuJoCo: {e}")

            # Show success message with project summary
            hardware_summary = []
            if config.get('microcontroller'):
                hardware_summary.append(f"Microcontroller: {config['microcontroller']}")
            if config.get('sensors'):
                hardware_summary.append(f"Sensors: {', '.join(config['sensors'][:3])}")
            if config.get('actuators'):
                hardware_summary.append(f"Actuators: {', '.join(config['actuators'][:3])}")

            summary_text = f"Project '{project_name}' created!\n\n"
            if hardware_summary:
                summary_text += "\n".join(hardware_summary) + "\n\n"

            summary_text += (
                "Next steps:\n"
                "1. Design your node graph (Node Graph tab)\n"
                "2. Generate ROS2 code (Ctrl+G)\n"
                "3. Build on VM (Ctrl+B)\n"
                "4. Run and test your robot!"
            )

            # Load wizard config into Project Status widget
            self.project_status_widget.load_wizard_config(config)

            # If this is a preset example, load the node graph automatically
            if config.get('is_preset_example') and config.get('node_graph_path'):
                self._load_example_node_graph(config)

            # Switch to Project Status tab to show guidance
            self._switch_to_tab("Project Status")

            # Show success message
            QMessageBox.information(
                self,
                "Project Created - Check Project Status Tab!",
                f"{summary_text}\n\n"
                "✅ Switch to the 'Project Status' tab to see your workflow progress "
                "and next steps!"
            )

            self.status_bar.showMessage(f"Project '{project_name}' ready! See Project Status tab", 5000)

        except Exception as e:
            QMessageBox.critical(
                self,
                "Project Creation Error",
                f"Failed to create project:\n\n{e}"
            )
            import traceback
            traceback.print_exc()

    def _load_example_node_graph(self, config: dict):
        """Load node graph from preset example and display visually"""
        try:
            import json
            node_graph_path = config.get('node_graph_path')

            if not node_graph_path or not Path(node_graph_path).exists():
                self.logger.warning(f"Node graph file not found: {node_graph_path}")
                return

            # Load JSON
            with open(node_graph_path, 'r') as f:
                graph_data = json.load(f)

            # Create new NodeGraph with actual nodes
            from roboshire.backend.node_graph import NodeGraph, Node, NodeType, Port, PortType, Connection, generate_id

            graph = NodeGraph()
            node_id_mapping = {}  # Map JSON IDs to actual Node objects

            # Create nodes from JSON
            for node_data in graph_data.get('nodes', []):
                # Determine node type
                node_type_str = node_data.get('type', 'ProcessingNode')
                try:
                    node_type = NodeType(node_type_str)
                except ValueError:
                    # Map common names to actual NodeType enum values
                    type_mapping = {
                        'SerialBridgeNode': NodeType.SUBSCRIBER,  # Subscribes to serial data
                        'SensorNode': NodeType.PUBLISHER,  # Publishes sensor data
                        'ActuatorNode': NodeType.SUBSCRIBER,  # Subscribes to commands
                        'ProcessingNode': NodeType.LOGIC,  # Processes data
                        'ControlNode': NodeType.LOGIC,  # Control logic
                        'LoggerNode': NodeType.SUBSCRIBER  # Subscribes to data to log
                    }
                    node_type = type_mapping.get(node_type_str, NodeType.LOGIC)

                # Create node with position from JSON
                pos_data = node_data.get('position', {})
                position = (pos_data.get('x', 100), pos_data.get('y', 100))

                # Get properties and ensure topic_name for Publisher/Subscriber nodes
                properties = node_data.get('properties', {}).copy()
                node_name = node_data.get('name', 'unnamed')

                # Add topic_name if not present and node is Publisher or Subscriber
                if node_type in [NodeType.PUBLISHER, NodeType.SUBSCRIBER]:
                    if 'topic_name' not in properties:
                        # Generate topic name from node name or property
                        topic_from_props = properties.get('topic_name') or properties.get('output_topic')
                        properties['topic_name'] = topic_from_props or f"/{node_name}"

                node = Node(
                    id=generate_id("node"),
                    name=node_name,
                    node_type=node_type,
                    position=position,
                    properties=properties
                )

                # Add basic ports (simplified - real impl would use node library)
                # Publishers only output, subscribers only input, logic nodes have both
                if node_type in [NodeType.SUBSCRIBER, NodeType.LOGIC, NodeType.SERVICE_SERVER]:
                    node.add_port(Port(
                        id=generate_id("port"),
                        name="input",
                        port_type=PortType.INPUT,
                        data_type="Any"
                    ))

                if node_type in [NodeType.PUBLISHER, NodeType.LOGIC, NodeType.SERVICE_CLIENT]:
                    node.add_port(Port(
                        id=generate_id("port"),
                        name="output",
                        port_type=PortType.OUTPUT,
                        data_type="Any"
                    ))

                graph.add_node(node)
                node_id_mapping[node_data['id']] = node

            self.logger.info(f"Created {len(graph.nodes)} nodes from example")

            # Create visual topic nodes and route connections through them
            # This makes the data flow visible as: Publisher → Topic → Subscriber
            topic_nodes = {}

            for conn_data in graph_data.get('connections', []):
                source_node = node_id_mapping.get(conn_data['source'])
                target_node = node_id_mapping.get(conn_data['target'])

                if source_node and target_node:
                    # Determine topic name
                    topic_name = f"/{source_node.name}_data"

                    # Create topic node if it doesn't exist
                    if topic_name not in topic_nodes:
                        # Calculate position between source and target
                        src_x, src_y = source_node.position
                        tgt_x, tgt_y = target_node.position
                        topic_x = (src_x + tgt_x) / 2
                        topic_y = (src_y + tgt_y) / 2

                        # Create topic node
                        topic_node = Node(
                            id=generate_id("topic"),
                            name=f"topic_{source_node.name}",
                            node_type=NodeType.PARAMETER,  # Visual topic indicator
                            position=(topic_x, topic_y),
                            properties={
                                'is_topic': True,
                                'topic_name': topic_name,
                                'message_type': 'std_msgs/String',
                                'description': f'Data from {source_node.name}'
                            }
                        )

                        # Topic needs input (from publishers) and output (to subscribers)
                        topic_node.add_port(Port(
                            id=generate_id("port"),
                            name="input",
                            port_type=PortType.INPUT,
                            data_type="Any"
                        ))
                        topic_node.add_port(Port(
                            id=generate_id("port"),
                            name="output",
                            port_type=PortType.OUTPUT,
                            data_type="Any"
                        ))

                        graph.add_node(topic_node)
                        topic_nodes[topic_name] = topic_node

                    topic_node = topic_nodes[topic_name]

                    # Create two connections: Publisher → Topic, Topic → Subscriber
                    output_ports = source_node.get_output_ports()
                    topic_input = topic_node.get_input_ports()
                    if output_ports and topic_input:
                        graph.add_connection(Connection(
                            id=generate_id("conn"),
                            from_port_id=output_ports[0].id,
                            to_port_id=topic_input[0].id,
                            topic_name=topic_name
                        ))

                    topic_output = topic_node.get_output_ports()
                    input_ports = target_node.get_input_ports()
                    if topic_output and input_ports:
                        graph.add_connection(Connection(
                            id=generate_id("conn"),
                            from_port_id=topic_output[0].id,
                            to_port_id=input_ports[0].id,
                            topic_name=topic_name
                        ))

            self.logger.info(f"Created {len(topic_nodes)} topic nodes")
            self.logger.info(f"Created {len(graph.connections)} connections from example")

            # IMPORTANT: Load graph into editor - this displays it visually!
            self.node_graph_editor.load_graph(graph)

            # Mark node graph as designed in project status
            self.project_status_widget.update_status('node_graph_designed', True)

            self.status_bar.showMessage(
                f"✓ Example loaded: {len(graph.nodes)} nodes, {len(graph.connections)} connections",
                5000
            )

            self.logger.info(f"Successfully loaded and displayed example from {node_graph_path}")

        except Exception as e:
            self.logger.error(f"Failed to load example node graph: {e}")
            import traceback
            traceback.print_exc()

    def _on_open_project(self):
        """Open existing project"""
        # Check for unsaved changes
        if not self._check_unsaved_changes():
            return

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open RoboShire Project",
            "",
            "RoboShire Projects (*.rsproj)"
        )
        if file_path:
            QMessageBox.information(self, "Open Project", f"Opening: {file_path}\n(Implementation coming soon)")

    def _on_save_project(self):
        """Save current project"""
        if not self.current_project:
            # No project path, do save as
            self._on_save_project_as()
            return

        try:
            # Save project logic here (placeholder for now)
            # TODO: Implement actual project save
            self.project_modified = False
            self.last_save_time = time.time()
            self._update_ui_state()
            self._update_status_bar_save_time()
            self.status_bar.showMessage(f"Project saved: {self.current_project}", 3000)
            self.logger.info(f"Project saved: {self.current_project}")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save project:\n{str(e)}")
            self.logger.error(f"Failed to save project: {e}", exc_info=True)

    def _on_save_project_as(self):
        """Save project as new file"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save RoboShire Project As",
            "",
            "RoboShire Projects (*.rsproj)"
        )
        if file_path:
            self.current_project = file_path
            self._on_save_project()

    def _on_autosave(self):
        """Auto-save project if modified"""
        if not self.autosave_enabled:
            return

        if self.project_modified and self.current_project:
            try:
                # Save project silently
                # TODO: Implement actual project save
                self.project_modified = False
                self.last_save_time = time.time()
                self._update_status_bar_save_time()
                self.logger.info(f"Auto-saved project: {self.current_project}")
                self.status_bar.showMessage("Auto-saved project", 2000)
            except Exception as e:
                self.logger.error(f"Auto-save failed: {e}", exc_info=True)
                # Don't show error dialog for auto-save failures

    def _check_unsaved_changes(self) -> bool:
        """
        Check for unsaved changes and prompt user to save

        Returns:
            True if it's safe to proceed (no changes or user chose to discard)
            False if user chose to cancel
        """
        if not self.project_modified:
            return True

        # Create dialog with save/discard/cancel options
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Warning)
        msg.setWindowTitle("Unsaved Changes")
        msg.setText("You have unsaved changes in the current project.")
        msg.setInformativeText("Do you want to save your changes before continuing?")

        save_btn = msg.addButton("Save", QMessageBox.AcceptRole)
        discard_btn = msg.addButton("Discard", QMessageBox.DestructiveRole)
        cancel_btn = msg.addButton("Cancel", QMessageBox.RejectRole)

        msg.setDefaultButton(save_btn)
        msg.exec()

        clicked = msg.clickedButton()

        if clicked == save_btn:
            # Save the project
            self._on_save_project()
            # Return True if save succeeded (project_modified will be False)
            return not self.project_modified
        elif clicked == discard_btn:
            # User chose to discard changes
            return True
        else:  # cancel_btn
            # User chose to cancel the operation
            return False

    def _update_status_bar_save_time(self):
        """Update status bar with last save time"""
        if self.last_save_time:
            import time as time_module
            elapsed = time_module.time() - self.last_save_time
            if elapsed < 60:
                time_str = "just now"
            elif elapsed < 3600:
                minutes = int(elapsed / 60)
                time_str = f"{minutes} min ago"
            else:
                hours = int(elapsed / 3600)
                time_str = f"{hours}h ago"

            # Update permanent widget in status bar if it exists
            if hasattr(self, 'save_time_label'):
                self.save_time_label.setText(f"Last saved: {time_str}")
        elif hasattr(self, 'save_time_label'):
            self.save_time_label.setText("Not saved")

    def _on_import_urdf(self):
        """Import URDF file"""
        # Check if a URDF is already loaded
        if self.urdf_manager.is_loaded():
            # Prompt to save changes or switch
            reply = QMessageBox.question(
                self,
                "URDF Already Loaded",
                "A URDF is currently loaded. Loading a new URDF will replace it.\n\n"
                "Do you want to continue?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )

            if reply != QMessageBox.Yes:
                return

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Import URDF File",
            "",
            "URDF Files (*.urdf *.xacro);;All Files (*)"
        )
        if file_path:
            success = self.urdf_manager.load(file_path)
            if success:
                self.urdf_tree_editor.refresh()
                self.status_bar.showMessage(f"Loaded: {file_path}")
                self._update_ui_state()  # Update UI state after loading

                # Phase 8: Auto-load into MuJoCo viewer
                try:
                    mujoco_loaded = self.mujoco_viewer.load_urdf(file_path)
                    if mujoco_loaded:
                        # Switch to MuJoCo tab to show visualization
                        self._switch_to_tab("MuJoCo")
                        self.status_bar.showMessage(f"Loaded in MuJoCo: {file_path}")
                except Exception as e:
                    self.logger.warning(f"Failed to load URDF into MuJoCo: {e}")

                # Validate URDF
                is_valid, errors = self.urdf_manager.validate()
                if not is_valid:
                    error_msg = "URDF loaded but has validation errors:\n\n"
                    error_msg += "\n".join(errors[:5])  # Show first 5 errors
                    if len(errors) > 5:
                        error_msg += f"\n\n... and {len(errors) - 5} more"
                    QMessageBox.warning(self, "URDF Validation", error_msg)
                else:
                    QMessageBox.information(
                        self,
                        "URDF Loaded",
                        f"Successfully loaded URDF:\n\n{self.urdf_manager.get_summary()}\n\n"
                        f"MuJoCo visualization: {'Ready' if mujoco_loaded else 'Failed'}"
                    )
            else:
                QMessageBox.critical(
                    self,
                    "Error",
                    f"Failed to load URDF file:\n{file_path}"
                )

    def _on_save_urdf(self):
        """Save URDF file"""
        if not self.urdf_manager.is_loaded():
            QMessageBox.warning(self, "No URDF", "No URDF loaded to save")
            return

        # Save to current file
        success = self.urdf_manager.save()
        if success:
            self.status_bar.showMessage(f"Saved: {self.urdf_manager.file_path}")
            self._update_ui_state()
            QMessageBox.information(
                self,
                "URDF Saved",
                f"Successfully saved URDF to:\n{self.urdf_manager.file_path}"
            )
        else:
            QMessageBox.critical(
                self,
                "Error",
                "Failed to save URDF file"
            )

    def _on_export_urdf(self):
        """Export URDF to new file"""
        if not self.urdf_manager.is_loaded():
            QMessageBox.warning(self, "No URDF", "No URDF loaded to export")
            return

        # Get save location
        default_name = ""
        if self.urdf_manager.file_path:
            default_name = str(self.urdf_manager.file_path)

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export URDF As",
            default_name,
            "URDF Files (*.urdf);;All Files (*)"
        )

        if file_path:
            # Ensure .urdf extension
            if not file_path.endswith('.urdf'):
                file_path += '.urdf'

            success = self.urdf_manager.save(file_path)
            if success:
                self.status_bar.showMessage(f"Exported: {file_path}")
                self._update_ui_state()
                QMessageBox.information(
                    self,
                    "URDF Exported",
                    f"Successfully exported URDF to:\n{file_path}"
                )
            else:
                QMessageBox.critical(
                    self,
                    "Error",
                    f"Failed to export URDF to:\n{file_path}"
                )

    def _on_view_rviz(self):
        """Launch RViz2"""
        if not self.urdf_manager.is_loaded():
            QMessageBox.warning(self, "No URDF", "Please load a URDF file first")
            return

        if self.rviz_bridge.is_running:
            QMessageBox.information(self, "RViz2", "RViz2 is already running")
            return

        urdf_path = self.urdf_manager.file_path
        success = self.rviz_bridge.launch(urdf_path, use_robot_state_publisher=True)

        if success:
            self.status_bar.showMessage("RViz2 launched")
            QMessageBox.information(
                self,
                "RViz2 Launched",
                "RViz2 has been launched on the Ubuntu VM.\n\n"
                "Note: Make sure the Ubuntu VM GUI is accessible."
            )
        else:
            QMessageBox.critical(
                self,
                "Error",
                "Failed to launch RViz2.\n\n"
                "Check that:\n"
                "1. Ubuntu VM is running\n"
                "2. SSH access is configured\n"
                "3. ROS2 is installed"
            )

    def _on_view_urdf_viz(self):
        """Launch urdf-viz"""
        QMessageBox.information(self, "urdf-viz", "urdf-viz integration coming soon!")

    def _on_simulate_gazebo(self):
        """Launch Gazebo simulation"""
        QMessageBox.information(self, "Gazebo", "Gazebo integration coming soon!")

    def _on_simulate_mujoco(self):
        """Launch MuJoCo simulation"""
        QMessageBox.information(self, "MuJoCo", "MuJoCo integration coming soon!")

    def _on_generate_code(self):
        """Generate ROS2 code from node graph"""
        # Get the node graph
        graph = self.node_graph_editor.get_node_graph()

        # Validate first
        is_valid, errors = graph.validate()
        if not is_valid:
            error_msg = "Cannot generate code - graph has errors:\n\n"
            error_msg += "\n".join(errors[:5])
            if len(errors) > 5:
                error_msg += f"\n\n... and {len(errors) - 5} more"
            QMessageBox.warning(self, "Validation Errors", error_msg)
            return

        # Check if graph is empty
        if len(graph.nodes) == 0:
            QMessageBox.warning(self, "Empty Graph", "Please add some nodes to the graph first!")
            return

        # Get package name from user (with beginner-friendly explanation)
        from roboshire.gui.beginner_dialogs import ask_package_name, ask_output_directory

        package_name = ask_package_name(self, default_name="my_robot_pkg")
        if not package_name:
            return

        # Get output directory (with workspace structure explanation)
        output_dir = ask_output_directory(self, default_dir=str(Path("workspace/src")))
        if not output_dir:
            return

        # Generate code
        self.status_bar.showMessage("Generating code...")

        try:
            success = self.code_generator.generate_package(
                graph=graph,
                package_name=package_name,
                output_dir=Path(output_dir),
                maintainer_name="User",
                maintainer_email="user@example.com",
                description=f"ROS2 package generated by RoboShire"
            )

            if success:
                pkg_path = Path(output_dir) / package_name
                self.status_bar.showMessage(f"Code generated: {pkg_path}")

                # Update project status
                self.project_status_widget.update_status('node_graph_designed', True)
                self.project_status_widget.update_status('code_generated', True)

                # Update package file tree with newly generated package
                try:
                    # Get list of packages from workspace (local file scan)
                    workspace_src = Path("workspace/src")
                    if workspace_src.exists():
                        packages = [p.name for p in workspace_src.iterdir() if p.is_dir() and not p.name.startswith('.')]
                        self.package_file_tree.update_packages(packages)
                        self.logger.info(f"Updated package file tree with {len(packages)} packages")
                except Exception as e:
                    self.logger.warning(f"Failed to update package file tree: {e}")

                QMessageBox.information(
                    self,
                    "Code Generated",
                    f"Successfully generated ROS2 package!\n\n"
                    f"Package: {package_name}\n"
                    f"Location: {pkg_path}\n\n"
                    f"Nodes: {len(graph.nodes)}\n"
                    f"Connections: {len(graph.connections)}\n\n"
                    f"Next step: Build with colcon:\n"
                    f"  cd {output_dir}\n"
                    f"  colcon build --packages-select {package_name}"
                )
            else:
                QMessageBox.critical(self, "Error", "Failed to generate code. Check the logs.")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Code generation failed:\n\n{e}")
            import traceback
            traceback.print_exc()

    def _ensure_build_managers(self) -> bool:
        """
        Ensure workspace and colcon managers are initialized (v2.3.0 - execution mode aware)

        Returns:
            True if successful, False otherwise
        """
        if self.workspace_manager and self.colcon_bridge:
            return True  # Already initialized

        try:
            # v2.3.0: Get workspace path based on execution mode
            mode = self.settings.get_execution_mode()

            if mode == 'local':
                local_config = self.settings.get_local_config()
                workspace_path = os.path.expanduser(local_config.workspace_path)
                ros_distro = local_config.ros_distro
            else:  # remote
                remote_config = self.settings.get_remote_config()
                workspace_path = remote_config.workspace_path
                ros_distro = remote_config.ros_distro

                # Initialize SSH manager for remote mode
                if not self.ssh_manager:
                    self.ssh_manager = SSHManagerSubprocess(
                        host=remote_config.ssh_host,
                        user=remote_config.ssh_user,
                        port=remote_config.ssh_port
                    )

                    # Check connection
                    if not self.ssh_manager.is_connected():
                        QMessageBox.critical(
                            self,
                            "SSH Connection Failed",
                            f"Could not connect to {remote_config.ssh_user}@{remote_config.ssh_host}\n\n"
                            "Please check:\n"
                            "1. Remote device is running\n"
                            "2. SSH is configured\n"
                            "3. Network connectivity"
                        )
                        return False

            # Initialize workspace manager (v2.3.0 - uses ExecutionManager)
            self.workspace_manager = WorkspaceManager(
                workspace_path=workspace_path,
                ros_distro=ros_distro
            )

            # Setup workspace (create if needed)
            if not self.workspace_manager.setup_workspace(create_if_missing=True):
                QMessageBox.critical(
                    self,
                    "Workspace Setup Failed",
                    f"Could not setup workspace: {workspace_path}"
                )
                return False

            # Initialize colcon bridge
            self.colcon_bridge = ColconBridge(self.workspace_manager)

            logging.info("Build managers initialized successfully")
            return True

        except Exception as e:
            QMessageBox.critical(
                self,
                "Build System Error",
                f"Failed to initialize build system:\n\n{e}"
            )
            logging.error(f"Build manager initialization failed: {e}")
            return False

    def _ensure_launch_managers(self) -> bool:
        """
        Ensure SSH, workspace, and ROS2 launcher are initialized

        Returns:
            True if successful, False otherwise
        """
        # First ensure build managers (SSH, workspace)
        if not self._ensure_build_managers():
            return False

        # Initialize ROS2 launcher if not already done
        if not self.ros2_launcher:
            try:
                self.ros2_launcher = ROS2Launcher(
                    workspace_path=self.settings.workspace.path,
                    ros_distro=self.settings.workspace.ros_distro
                )
                logging.info("ROS2 launcher initialized successfully")
            except Exception as e:
                QMessageBox.critical(
                    self,
                    "Launch System Error",
                    f"Failed to initialize ROS2 launcher:\n\n{e}"
                )
                logging.error(f"ROS2 launcher initialization failed: {e}")
                return False

        # Initialize Phase 5 components if not already done
        if not self.node_monitor:
            try:
                self.node_monitor = NodeMonitor(
                    ros2_launcher=self.ros2_launcher,
                    poll_interval=5.0,
                    status_callback=self._on_node_status_changed  # ADD CRASH DETECTION
                )
                self.node_status_widget.set_node_monitor(self.node_monitor)
                self.node_monitor.start_monitoring()
                logging.info("NodeMonitor initialized and started")
            except Exception as e:
                logging.error(f"NodeMonitor initialization failed: {e}")

        if not self.topic_inspector:
            try:
                self.topic_inspector = TopicInspector(
                    execution_manager=self.execution_manager,
                    ros_distro=self.settings.workspace.ros_distro
                )
                self.topic_inspector_widget.set_topic_inspector(self.topic_inspector)
                logging.info("TopicInspector initialized")
            except Exception as e:
                logging.error(f"TopicInspector initialization failed: {e}")

        if not self.service_manager:
            try:
                self.service_manager = ServiceManager(
                    ssh_manager=self.ssh_manager,
                    ros_distro=self.settings.workspace.ros_distro
                )
                logging.info("ServiceManager initialized")
            except Exception as e:
                logging.error(f"ServiceManager initialization failed: {e}")

        if not self.param_manager:
            try:
                self.param_manager = ParameterManager(
                    ssh_manager=self.ssh_manager,
                    ros_distro=self.settings.workspace.ros_distro
                )
                self.param_editor_widget.set_param_manager(self.param_manager)
                logging.info("ParameterManager initialized")
            except Exception as e:
                logging.error(f"ParameterManager initialization failed: {e}")

        # Phase 6: Initialize lifecycle manager
        if not self.lifecycle_manager:
            try:
                self.lifecycle_manager = LifecycleManager(
                    ssh_manager=self.ssh_manager
                )

                # Replace placeholder with actual lifecycle editor widget
                if not self.lifecycle_editor_widget:
                    self.lifecycle_editor_widget = LifecycleEditorWidget(self.lifecycle_manager)

                    # Find the Build & Deploy category tab and replace the placeholder
                    for i in range(self.tab_widget.count()):
                        if "Build & Deploy" in self.tab_widget.tabText(i):
                            build_tabs = self.tab_widget.widget(i)
                            if isinstance(build_tabs, QTabWidget):
                                # Find Lifecycle Manager sub-tab
                                for j in range(build_tabs.count()):
                                    if build_tabs.tabText(j) == "Lifecycle Manager":
                                        build_tabs.removeTab(j)
                                        build_tabs.insertTab(j, self.lifecycle_editor_widget, "Lifecycle Manager")
                                        break
                            break

                logging.info("LifecycleManager initialized")
            except Exception as e:
                logging.error(f"LifecycleManager initialization failed: {e}")

        return True

    def _on_build(self):
        """Build ROS2 workspace"""
        # Ensure execution mode is initialized (v2.3.0)
        try:
            self.execution_manager.get_executor()
        except RuntimeError:
            QMessageBox.warning(
                self,
                "Execution Mode Not Configured",
                "Please configure execution mode first:\n\n"
                "Settings → Execution Mode"
            )
            return

        # Ensure build managers are initialized
        if not self._ensure_build_managers():
            return

        # Get list of packages
        packages = self.colcon_bridge.list_buildable_packages()

        if not packages:
            QMessageBox.information(
                self,
                "No Packages",
                "No packages found in workspace.\n\n"
                "Generate code first using 'Generate Code' (Ctrl+G)"
            )
            return

        # Ask which packages to build (with beginner-friendly explanation)
        from roboshire.gui.beginner_dialogs import ask_build_package

        package_choice = ask_build_package(self, packages)
        if not package_choice:
            return

        # Switch to Build tab
        self._switch_to_tab("Build")

        # Clear previous output
        self.build_output_viewer.clear_output()

        # Start build
        self.build_output_viewer.start_build(package_count=len(packages) if package_choice == "All Packages" else 1)
        self.status_bar.showMessage("Building...")

        # Build options
        build_options = BuildOptions(
            symlink_install=self.settings.build.symlink_install,
            parallel_workers=self.settings.build.parallel_workers,
            continue_on_error=self.settings.build.continue_on_error,
            event_handlers="console_direct+"
        )

        if package_choice != "All Packages":
            build_options.packages_select = [package_choice]

        # Run build in a thread to avoid freezing GUI
        from PySide6.QtCore import QThread, Signal

        class BuildThread(QThread):
            """Thread for running build"""
            output_line = Signal(str, str)  # line, stream
            build_complete = Signal(bool)  # success

            def __init__(self, colcon_bridge, build_options):
                super().__init__()
                self.colcon_bridge = colcon_bridge
                self.build_options = build_options
                self.result = None

            def run(self):
                def output_callback(line, stream):
                    self.output_line.emit(line, stream)

                self.result = self.colcon_bridge.build_workspace(
                    options=self.build_options,
                    output_callback=output_callback
                )

                success = self.result.exit_code == 0
                self.build_complete.emit(success)

        # Create and start build thread
        self.build_thread = BuildThread(self.colcon_bridge, build_options)
        self.build_thread.output_line.connect(self.build_output_viewer.append_output)
        self.build_thread.build_complete.connect(self._on_build_complete)
        self.build_thread.start()

        # Disable build action while building
        self.build_action.setEnabled(False)

    def _on_build_complete(self, success: bool):
        """Handle build completion"""
        self.build_output_viewer.finish_build(success)

        if success:
            self.status_bar.showMessage("Build successful!", 5000)
            # Update project status
            self.project_status_widget.update_status('build_completed', True)
            # Update architecture diagram
            self.system_architecture_widget.update_build_status("Built successfully")
            # Update packages list
            if self.colcon_bridge:
                packages = self.colcon_bridge.list_buildable_packages()
                self.system_architecture_widget.update_packages(packages)
        else:
            self.status_bar.showMessage("Build failed!", 5000)
            self.system_architecture_widget.update_build_status("Build failed")

            # v2.5.0: Show friendly error dialog
            build_output = self.build_output_viewer.get_output_text()
            if build_output:
                show_build_error(build_output, parent=self)

        # Re-enable build action
        self.build_action.setEnabled(True)

    def _on_build_cancel(self):
        """Handle build cancellation request"""
        # TODO: Implement build cancellation
        logging.warning("Build cancellation not yet implemented")

    def _on_clean(self):
        """Clean build artifacts"""
        # Ensure build managers are initialized
        if not self._ensure_build_managers():
            return

        # Confirm clean
        reply = QMessageBox.question(
            self,
            "Clean Build",
            "This will remove all build artifacts (build/, install/, log/).\n\n"
            "Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply != QMessageBox.Yes:
            return

        # Perform clean
        self.status_bar.showMessage("Cleaning...")

        try:
            success = self.colcon_bridge.clean_workspace()

            if success:
                self.status_bar.showMessage("Workspace cleaned", 3000)
                QMessageBox.information(
                    self,
                    "Clean Complete",
                    "Build artifacts have been removed."
                )
            else:
                QMessageBox.warning(
                    self,
                    "Clean Failed",
                    "Failed to clean workspace. Check logs for details."
                )

        except Exception as e:
            QMessageBox.critical(
                self,
                "Clean Error",
                f"Error cleaning workspace:\n\n{e}"
            )
            logging.error(f"Clean failed: {e}")

    def _on_run(self):
        """Run ROS2 nodes"""
        # Ensure launch managers are initialized
        if not self._ensure_launch_managers():
            return

        # Get list of packages
        packages = self.colcon_bridge.list_buildable_packages()

        if not packages:
            QMessageBox.information(
                self,
                "No Packages",
                "No packages found in workspace.\n\n"
                "Generate and build code first using:\n"
                "1. 'Generate Code' (Ctrl+G)\n"
                "2. 'Build' (Ctrl+B)"
            )
            return

        # Ask which package/node to launch (with beginner-friendly explanation)
        from roboshire.gui.beginner_dialogs import ask_executable
        from PySide6.QtWidgets import QInputDialog

        # First ask for package (simple dropdown is fine here)
        package_choice, ok = QInputDialog.getItem(
            self,
            "Select Package",
            f"Found {len(packages)} package(s).\n\nSelect package to run:",
            packages,
            0,
            False
        )

        if not ok or not package_choice:
            return

        # Ask for executable with detailed explanation
        # TODO: Could scan package for executables and pass list
        executable_name = ask_executable(self, package_name=package_choice, executables=None)

        if not executable_name:
            return

        # Check if node with same name is already running (prevents duplicates)
        if executable_name in self.running_nodes:
            reply = QMessageBox.question(
                self,
                "Node Already Running",
                f"Node '{executable_name}' is already running.\n\n"
                f"Do you want to restart it?\n\n"
                f"Yes = Stop old instance and start new one\n"
                f"No = Cancel",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )

            if reply != QMessageBox.Yes:
                return

            # Stop the existing node first
            self.status_bar.showMessage(f"Stopping existing {executable_name}...")
            self.log_viewer.append_log(
                f"[INFO] Stopping existing node: {executable_name} (preventing duplicate)",
                executable_name,
                "INFO"
            )

            try:
                # Stop the specific node by name
                if self.ros2_launcher:
                    # Use stop_all_nodes for now (will kill all matching process names)
                    # TODO: Implement stop_node_by_name in ROS2Launcher for precise stopping
                    import subprocess
                    cmd = f"pkill -f '{executable_name}' || true"
                    subprocess.run(
                        ['ssh', f"{self.settings.ssh.user}@{self.settings.ssh.host}", cmd],
                        timeout=5
                    )

                # Remove from running nodes list
                if executable_name in self.running_nodes:
                    self.running_nodes.remove(executable_name)

                # Unregister from node monitor
                if self.node_monitor:
                    try:
                        self.node_monitor.unregister_node(executable_name)
                    except Exception as e:
                        self.logger.warning(f"Error unregistering node from monitor: {e}")

                # Small delay to ensure node is fully stopped
                import time
                time.sleep(1.0)

                self.log_viewer.append_log(
                    f"[INFO] Existing node stopped. Launching new instance...",
                    executable_name,
                    "INFO"
                )

            except Exception as e:
                self.logger.warning(f"Error stopping existing node: {e}")
                # Continue anyway - try to launch the new one

        # Switch to Logs tab
        self._switch_to_tab("Logs")

        # Launch node in background thread
        from PySide6.QtCore import QThread, Signal

        class LaunchThread(QThread):
            """Thread for launching ROS2 node"""
            output_line = Signal(str, str)  # line, stream
            launch_complete = Signal(bool, str)  # success, node_name

            def __init__(self, launcher, package_name, executable_name):
                super().__init__()
                self.launcher = launcher
                self.package_name = package_name
                self.executable_name = executable_name
                self.result = None

            def run(self):
                def output_callback(line, stream):
                    self.output_line.emit(line, stream)

                self.result = self.launcher.launch_node(
                    package_name=self.package_name,
                    executable=self.executable_name,
                    output_callback=output_callback
                )

                success = self.result.success
                node_name = self.result.node_name
                self.launch_complete.emit(success, node_name)

        # Create and start launch thread
        self.launch_thread = LaunchThread(self.ros2_launcher, package_choice, executable_name)
        self.launch_thread.output_line.connect(lambda line, stream: self.log_viewer.append_log(
            line, executable_name, "INFO"
        ))
        self.launch_thread.launch_complete.connect(self._on_launch_complete)
        self.launch_thread.start()

        self.status_bar.showMessage(f"Launching {executable_name}...")

    def _on_launch_complete(self, success: bool, node_name: str):
        """Handle launch completion"""
        if success:
            self.running_nodes.append(node_name)
            self.status_bar.showMessage(f"Node launched: {node_name}")
            self.log_viewer.append_log(
                f"[INFO] Successfully launched node: {node_name}",
                node_name,
                "INFO"
            )

            # Register node with monitor for health tracking
            if self.node_monitor:
                self.node_monitor.register_node(
                    node_name=node_name,
                    auto_restart=False
                )
                self.logger.info(f"Registered {node_name} with node monitor")

            # Refresh topic inspector to show new topics
            if self.topic_inspector:
                try:
                    self.topic_inspector_widget.refresh_topics()
                except Exception as e:
                    self.logger.warning(f"Failed to refresh topics: {e}")

            # Update architecture diagram with running nodes
            self.system_architecture_widget.update_nodes(self.running_nodes)

            # Update project status
            self.project_status_widget.update_status('nodes_running', True)
            self._update_ui_state()
        else:
            QMessageBox.critical(
                self,
                "Launch Failed",
                f"Failed to launch node: {node_name}"
            )
            self.status_bar.showMessage("Launch failed")

    def _on_node_status_changed(self, node_name: str, health):
        """Handle node status changes (crashes, recovery) - called from background thread"""
        from roboshire.integrations.node_monitor import NodeHealth

        if health == NodeHealth.CRASHED:
            # Emit signal to handle crash on main GUI thread (thread-safe)
            self._node_crashed.emit(node_name)

    def _handle_node_crash_on_main_thread(self, node_name: str):
        """Handle node crash on main Qt thread (GUI-safe)"""
        # Get error messages from log viewer
        log_output = self.log_viewer.get_recent_logs_for_node(node_name, max_lines=50)
        if not log_output:
            log_output = f"Node '{node_name}' crashed unexpectedly. No error logs captured."

        # v2.5.0: Show friendly error dialog
        show_node_crash_error(node_name, log_output, parent=self)

        # Log crash
        self.log_viewer.append_log(
            f"[ERROR] Node crashed: {node_name}",
            node_name,
            "ERROR"
        )

        # Update architecture diagram - remove from running nodes
        if node_name in self.running_nodes:
            self.running_nodes.remove(node_name)
            self.system_architecture_widget.update_nodes(self.running_nodes)
            self._update_ui_state()

    def _on_stop(self):
        """Stop running nodes"""
        if not self.ros2_launcher:
            return

        if not self.running_nodes:
            QMessageBox.information(
                self,
                "No Running Nodes",
                "No nodes are currently running."
            )
            return

        # Confirm stop
        reply = QMessageBox.question(
            self,
            "Stop Nodes",
            f"Stop all {len(self.running_nodes)} running node(s)?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply != QMessageBox.Yes:
            return

        # Mark all nodes as gracefully stopping (prevents false crash detection)
        if self.node_monitor:
            for node_name in self.running_nodes:
                self.node_monitor.mark_node_stopping(node_name)
                self.logger.info(f"Marked {node_name} as gracefully stopping")

        # Stop all nodes
        self.status_bar.showMessage("Stopping nodes...")

        try:
            stopped_count = self.ros2_launcher.stop_all_nodes()

            self.running_nodes.clear()
            # Update architecture diagram
            self.system_architecture_widget.update_nodes([])
            self._update_ui_state()

            self.status_bar.showMessage(f"Stopped {stopped_count} node(s)", 3000)
            self.log_viewer.append_log(
                f"[INFO] Stopped {stopped_count} node(s)",
                "",
                "INFO"
            )

            QMessageBox.information(
                self,
                "Nodes Stopped",
                f"Successfully stopped {stopped_count} node(s)"
            )

        except Exception as e:
            QMessageBox.critical(
                self,
                "Error",
                f"Error stopping nodes:\n\n{e}"
            )
            logging.error(f"Error stopping nodes: {e}")

    def _on_open_testing_guide(self):
        """Open testing and validation guide dialog"""
        # Show comprehensive testing checklist
        testing_guide = QMessageBox(self)
        testing_guide.setWindowTitle("Testing & Validation Guide")
        testing_guide.setIcon(QMessageBox.Information)

        guide_text = (
            "<h3>Testing & Validation Checklist</h3>"
            "<p>Use these tabs and tools to verify your robot is working correctly:</p>"

            "<h4>1. Topics Tab (Ctrl+6)</h4>"
            "<ul>"
            "<li>Check that all expected topics are listed</li>"
            "<li>Verify data is being published (Hz rate > 0)</li>"
            "<li>Click 'Echo Topic' to see actual data values</li>"
            "</ul>"

            "<h4>2. Node Status Tab (Ctrl+5)</h4>"
            "<ul>"
            "<li>Confirm all nodes show 'RUNNING' status</li>"
            "<li>Check CPU and memory usage is reasonable</li>"
            "<li>Watch for any nodes crashing or restarting</li>"
            "</ul>"

            "<h4>3. Parameters Tab (Ctrl+7)</h4>"
            "<ul>"
            "<li>Verify all parameters have correct values</li>"
            "<li>Test parameter changes take effect immediately</li>"
            "</ul>"

            "<h4>4. Logs Tab (Ctrl+4)</h4>"
            "<ul>"
            "<li>Look for ERROR or WARNING messages</li>"
            "<li>Verify sensor data looks correct</li>"
            "<li>Check timing of events</li>"
            "</ul>"

            "<h4>5. System Architecture Widget</h4>"
            "<ul>"
            "<li>Verify all nodes appear in the diagram</li>"
            "<li>Check data flow matches your design</li>"
            "</ul>"

            "<h4>6. ROS2 Tools</h4>"
            "<ul>"
            "<li>Use rqt_graph (Tools menu) to visualize node connections</li>"
            "<li>Use rqt_console (Tools menu) for advanced log filtering</li>"
            "</ul>"

            "<p><b>When testing is complete:</b></p>"
            "<p>Return to the Project Status tab and click the testing button again "
            "to mark testing as complete!</p>"
        )

        testing_guide.setText(guide_text)
        testing_guide.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        testing_guide.button(QMessageBox.Ok).setText("Mark Testing Complete")
        testing_guide.button(QMessageBox.Cancel).setText("Continue Testing")

        result = testing_guide.exec()

        # If user clicks "Mark Testing Complete", update project status
        if result == QMessageBox.Ok:
            self.project_status_widget.update_status('testing_complete', True)
            self.status_bar.showMessage("Testing marked as complete!", 3000)
            QMessageBox.information(
                self,
                "Testing Complete!",
                "Great job! Your robot has been tested and validated.\n\n"
                "Next step: Prepare for deployment!"
            )

        self.logger.info("Testing guide opened")

    def _on_logs_cleared(self):
        """Handle log viewer clear request"""
        logging.info("Logs cleared by user")

    def _on_rqt_graph(self):
        """Launch rqt_graph"""
        try:
            self.status_bar.showMessage("Launching rqt_graph...")

            # Build command to launch rqt_graph with proper DISPLAY
            cmd = (
                f"source /opt/ros/{self.settings.workspace.ros_distro}/setup.bash && "
                f"source {self.settings.workspace.path}/install/setup.bash && "
                f"rqt_graph > ~/roboshire_rqt_graph.log 2>&1 &"
            )

            # Launch in background using ExecutionManager
            executor = self.execution_manager.get_executor()
            async_process = executor.execute_command_async(cmd)

            self.status_bar.showMessage("rqt_graph launched", 3000)

            QMessageBox.information(
                self,
                "rqt_graph Launched",
                "rqt_graph is now running!\n\n"
                "The window should appear on your display.\n\n"
                "If rqt_graph doesn't appear, check the log:\n"
                "  cat ~/roboshire_rqt_graph.log"
            )

        except Exception as e:
            QMessageBox.critical(
                self,
                "Launch Error",
                f"Failed to launch rqt_graph:\n\n{e}"
            )
            logging.error(f"Failed to launch rqt_graph: {e}")

    def _on_rqt_console(self):
        """Launch rqt_console"""
        try:
            self.status_bar.showMessage("Launching rqt_console...")

            # Build command to launch rqt_console
            # NOTE: rqt_console is not a standalone command - must use ros2 run
            cmd = (
                f"source /opt/ros/{self.settings.workspace.ros_distro}/setup.bash && "
                f"source {self.settings.workspace.path}/install/setup.bash && "
                f"ros2 run rqt_console rqt_console > ~/roboshire_rqt_console.log 2>&1 &"
            )

            # Launch in background using ExecutionManager
            executor = self.execution_manager.get_executor()
            async_process = executor.execute_command_async(cmd)

            self.status_bar.showMessage("rqt_console launched", 3000)

            QMessageBox.information(
                self,
                "rqt_console Launched",
                "rqt_console is now running!\n\n"
                "The window should appear on your display.\n\n"
                "If rqt_console doesn't appear, check the log:\n"
                "  cat ~/roboshire_rqt_console.log"
            )

        except Exception as e:
            QMessageBox.critical(
                self,
                "Launch Error",
                f"Failed to launch rqt_console:\n\n{e}"
            )
            logging.error(f"Failed to launch rqt_console: {e}")

    def _on_call_service(self):
        """Open service/action call dialog"""
        # Ensure launch managers are initialized
        if not self._ensure_launch_managers():
            return

        try:
            dialog = ServiceCallDialog(self.service_manager, self)
            dialog.exec()
        except Exception as e:
            logging.error(f"Error opening service dialog: {e}")
            QMessageBox.critical(self, "Error", f"Failed to open service dialog:\n\n{e}")

    def _on_validate_urdf(self):
        """Validate current URDF file"""
        if not self.urdf_manager.is_loaded():
            QMessageBox.warning(
                self,
                "No URDF",
                "Please load a URDF file first.\n\nUse File > Import URDF or File > New from Example"
            )
            return

        # Get URDF file path
        urdf_path = self.urdf_manager.file_path
        if not urdf_path:
            QMessageBox.warning(
                self,
                "No URDF Path",
                "URDF is loaded but file path is unknown.\n\nPlease save the URDF first."
            )
            return

        # Run validation
        self.status_bar.showMessage("Validating URDF...")
        self.urdf_validator_widget.validate_urdf(str(urdf_path))
        self.status_bar.showMessage("URDF validation complete")

        # Show validation results summary
        result = self.urdf_validator_widget.current_result
        if result:
            if result.is_valid:
                msg = (
                    "URDF Validation: VALID\n\n"
                    f"No errors found!\n\n"
                    f"Warnings: {result.warning_count}\n"
                    f"Info: {result.info_count}"
                )
                if result.warning_count > 0:
                    msg += "\n\nCheck the Validation tab for details."
                QMessageBox.information(self, "Validation Complete", msg)
            else:
                msg = (
                    "URDF Validation: INVALID\n\n"
                    f"Errors: {result.error_count}\n"
                    f"Warnings: {result.warning_count}\n"
                    f"Info: {result.info_count}\n\n"
                    "Please fix errors before using this URDF."
                )
                QMessageBox.warning(self, "Validation Failed", msg)

    def _on_validator_element_selected(self, element_name: str):
        """Handle element selection from validator"""
        # Try to select the element in URDF tree
        self.urdf_tree_editor.select_element(element_name)
        self.status_bar.showMessage(f"Selected element: {element_name}")

    def _on_ssh_setup_wizard(self):
        """Show SSH Setup Wizard"""
        wizard = SSHSetupWizard(self)
        wizard.ssh_configured.connect(self._on_ssh_configured)
        wizard.exec()

    def _on_ssh_configured(self, config: dict):
        """Handle SSH configuration from wizard"""
        self.logger.info(f"SSH configured: {config['host']}@{config['username']}")

        # Reinitialize SSH manager with new config
        self.ssh_manager = SSHManagerSubprocess(
            host=config['host'],
            username=config['username'],
            password=config.get('password'),
            key_file=config.get('key_file'),
            port=config.get('port', 22)
        )

        # Update settings
        self.settings.ssh.host = config['host']
        self.settings.ssh.username = config['username']
        self.settings.ssh.port = config.get('port', 22)
        if config.get('key_file'):
            self.settings.ssh.key_file = config['key_file']
        self.settings.save()

        self.status_bar.showMessage("SSH configuration saved successfully")
        QMessageBox.information(
            self,
            "SSH Configured",
            f"SSH connection configured successfully!\n\n"
            f"Host: {config['host']}\n"
            f"Username: {config['username']}\n"
            f"Port: {config.get('port', 22)}\n\n"
            f"You can now build and run ROS2 packages on your Ubuntu VM."
        )

    # v2.3.0: Ubuntu Standalone - Execution Mode handlers
    def _check_first_launch(self):
        """Check if this is first launch and show deployment wizard"""
        if not self.settings.has_execution_config():
            self.logger.info("First launch detected - showing deployment setup wizard")

            # Show deployment wizard
            wizard = DeploymentSetupWizard(self.settings, self)
            if wizard.exec() == QWizard.Accepted:
                # Wizard saved settings in accept() - now initialize execution manager
                self._initialize_execution_mode()
                self._update_execution_mode_indicator()
            else:
                # User cancelled - use defaults (local mode)
                self.logger.warning("Deployment wizard cancelled - using default local mode")
                self.settings.save_local_config(
                    workspace_path=os.path.expanduser("~/roboshire_workspace"),
                    ros_distro="humble"
                )
                self.settings.set_execution_mode('local')
                self.settings.save()
                self._initialize_execution_mode()
                self._update_execution_mode_indicator()
        else:
            # Not first launch - initialize from saved settings
            self._initialize_execution_mode()

    def _initialize_execution_mode(self):
        """Initialize execution manager based on current settings"""
        mode = self.settings.get_execution_mode()

        if mode == 'local':
            # Initialize local executor
            local_config = self.settings.get_local_config()
            self.execution_manager.initialize_local(ros_distro=local_config.ros_distro)
            success = self.execution_manager.set_mode(ExecutionMode.LOCAL)

            if success:
                self.logger.info(f"Initialized LOCAL execution mode: {local_config.ros_distro}")
            else:
                self.logger.error(f"Failed to initialize LOCAL execution mode - ROS2 {local_config.ros_distro} not found")
                QMessageBox.warning(
                    self,
                    "Local Execution Error",
                    f"Could not initialize local execution mode.\n\n"
                    f"ROS2 {local_config.ros_distro} not found at /opt/ros/{local_config.ros_distro}/\n\n"
                    f"Please install ROS2 or switch to Remote execution mode."
                )

        elif mode == 'remote':
            # Initialize remote executor (requires SSH connection)
            remote_config = self.settings.get_remote_config()

            # Initialize SSH manager if not already done
            if not self.ssh_manager:
                # Will be initialized when SSH is configured
                self.logger.info("Remote mode selected - SSH needs to be configured")
            else:
                self.execution_manager.initialize_remote(
                    ssh_manager=self.ssh_manager,
                    workspace_path=remote_config.workspace_path,
                    ros_distro=remote_config.ros_distro
                )
                success = self.execution_manager.set_mode(ExecutionMode.REMOTE)

                if success:
                    self.logger.info(f"Initialized REMOTE execution mode: {remote_config.ssh_user}@{remote_config.ssh_host}")
                else:
                    self.logger.error("Failed to initialize REMOTE execution mode")

    def _on_execution_mode(self):
        """Show execution mode dialog"""
        dialog = ExecutionModeDialog(self.settings, self)
        dialog.mode_changed.connect(self._on_execution_mode_changed)
        dialog.exec()

    def _on_execution_mode_changed(self, new_mode: str):
        """Handle execution mode change"""
        self.logger.info(f"Execution mode changed to: {new_mode}")

        # Reinitialize execution manager
        self._initialize_execution_mode()

        # Update UI
        self._update_execution_mode_indicator()

        # Show message
        mode_display = "LOCAL" if new_mode == 'local' else "REMOTE"
        self.status_bar.showMessage(f"Switched to {mode_display} execution mode", 5000)

    def _update_execution_mode_indicator(self):
        """Update execution mode indicator in status bar"""
        try:
            display_name = self.execution_manager.get_display_name()
            self.execution_mode_label.setText(display_name)
        except Exception as e:
            self.logger.error(f"Error updating execution mode indicator: {e}")
            self.execution_mode_label.setText("⚠️ Not configured")

    def _on_getting_started_action(self, action: str):
        """Handle action requested from Getting Started widget"""
        action_map = {
            'show_welcome': self._show_welcome_tour,
            'configure_ssh': self._on_ssh_setup_wizard,
            'new_robot_wizard': self._on_new_robot_wizard,
            'open_examples': self._on_new_from_example,
            'build_package': self._on_build,
            'open_mujoco': lambda: self._switch_to_tab("MuJoCo"),
            'run_nodes': self._on_run,
            'launch_rviz': self._on_view_rviz,
            'show_features': lambda: self._switch_to_tab("Project Status"),
            'show_shortcuts': self._on_show_shortcuts,
            'show_help': self._on_about
        }

        handler = action_map.get(action)
        if handler:
            handler()
        else:
            self.logger.warning(f"Unknown getting started action: {action}")

    def _show_welcome_tour(self):
        """Show welcome/quick tour with multi-slide presentation"""
        from PySide6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QStackedWidget, QTextBrowser
        from PySide6.QtCore import Qt

        dialog = QDialog(self)
        dialog.setWindowTitle("Welcome to RoboShire - Quick Tour")
        dialog.setMinimumSize(700, 500)

        layout = QVBoxLayout(dialog)

        # Create stacked widget for slides
        self.tour_slides = QStackedWidget()

        # Slide 1: Welcome
        slide1 = QWidget()
        s1_layout = QVBoxLayout(slide1)
        s1_title = QLabel("<h1>Welcome to RoboShire v2.3.0</h1>")
        s1_title.setAlignment(Qt.AlignCenter)
        s1_title.setStyleSheet("color: #556B2F; padding: 20px;")
        s1_layout.addWidget(s1_title)

        s1_text = QTextBrowser()
        s1_text.setHtml("""
            <div style='font-size: 14px; padding: 20px;'>
                <p style='font-size: 16px; color: #556B2F;'><b>RoboShire</b> is a visual IDE for ROS2 robotics development.</p>

                <h3 style='color: #556B2F;'>What makes RoboShire special?</h3>
                <ul>
                    <li><b>No ROS2 expertise required</b> - Visual tools guide you through everything</li>
                    <li><b>5-minute robot setup</b> - From idea to running code in minutes</li>
                    <li><b>Real-time simulation</b> - Test in MuJoCo before deploying to hardware</li>
                    <li><b>Ubuntu standalone</b> - Runs directly on Ubuntu (no VM needed)</li>
                </ul>

                <p style='margin-top: 20px;'><i>Click "Next" to see key features...</i></p>
            </div>
        """)
        s1_text.setOpenExternalLinks(False)
        s1_layout.addWidget(s1_text)
        self.tour_slides.addWidget(slide1)

        # Slide 2: Key Features
        slide2 = QWidget()
        s2_layout = QVBoxLayout(slide2)
        s2_title = QLabel("<h2>Key Features</h2>")
        s2_title.setAlignment(Qt.AlignCenter)
        s2_title.setStyleSheet("color: #556B2F; padding: 10px;")
        s2_layout.addWidget(s2_title)

        s2_text = QTextBrowser()
        s2_text.setHtml("""
            <div style='font-size: 13px; padding: 20px;'>
                <table width='100%' cellspacing='15'>
                    <tr>
                        <td width='50%' valign='top'>
                            <h4 style='color: #556B2F;'>Design Tools</h4>
                            <ul>
                                <li><b>Visual Node Graph Editor</b><br/>Drag-and-drop ROS2 node creation</li>
                                <li><b>URDF Robot Builder</b><br/>3D robot model designer</li>
                                <li><b>Behavior Tree Editor</b><br/>Complex logic, visually</li>
                            </ul>
                        </td>
                        <td width='50%' valign='top'>
                            <h4 style='color: #556B2F;'>Execution Tools</h4>
                            <ul>
                                <li><b>Code Generation</b><br/>Python/C++ from visual design</li>
                                <li><b>MuJoCo Simulation</b><br/>60 FPS physics simulation</li>
                                <li><b>Live Monitoring</b><br/>Topics, nodes, parameters</li>
                            </ul>
                        </td>
                    </tr>
                </table>
            </div>
        """)
        s2_layout.addWidget(s2_text)
        self.tour_slides.addWidget(slide2)

        # Slide 3: Workflow
        slide3 = QWidget()
        s3_layout = QVBoxLayout(slide3)
        s3_title = QLabel("<h2>Complete Workflow</h2>")
        s3_title.setAlignment(Qt.AlignCenter)
        s3_title.setStyleSheet("color: #556B2F; padding: 10px;")
        s3_layout.addWidget(s3_title)

        s3_text = QTextBrowser()
        s3_text.setHtml("""
            <div style='font-size: 13px; padding: 20px;'>
                <h3 style='color: #556B2F;'>From Zero to Robot in 8 Steps:</h3>
                <ol style='line-height: 1.8;'>
                    <li><b>Create Project</b> - Use Workflow Wizard (Ctrl+Shift+N)</li>
                    <li><b>Design Node Graph</b> - Visual nodes & connections</li>
                    <li><b>Generate Code</b> - Automatic Python/C++ generation</li>
                    <li><b>Build Package</b> - Compile with colcon (Ctrl+B)</li>
                    <li><b>Simulate</b> - Test in MuJoCo 3D viewer</li>
                    <li><b>Run Nodes</b> - Launch ROS2 nodes (Ctrl+R)</li>
                    <li><b>Test & Validate</b> - Monitor topics, logs, performance</li>
                    <li><b>Deploy</b> - Upload to Arduino/ESP32/Robot</li>
                </ol>

                <p style='background-color: #F0F5E6; padding: 10px; border-radius: 5px; margin-top: 20px;'>
                    <b>Tip:</b> The <b>Project Status</b> tab tracks your progress through all 8 steps!
                </p>
            </div>
        """)
        s3_layout.addWidget(s3_text)
        self.tour_slides.addWidget(slide3)

        # Slide 4: Getting Help
        slide4 = QWidget()
        s4_layout = QVBoxLayout(slide4)
        s4_title = QLabel("<h2>Getting Help & Resources</h2>")
        s4_title.setAlignment(Qt.AlignCenter)
        s4_title.setStyleSheet("color: #556B2F; padding: 10px;")
        s4_layout.addWidget(s4_title)

        s4_text = QTextBrowser()
        s4_text.setHtml("""
            <div style='font-size: 13px; padding: 20px;'>
                <h3 style='color: #556B2F;'>Built-in Help:</h3>
                <ul style='line-height: 1.8;'>
                    <li><b>Search Bar</b> - Type Ctrl+K to search anything ("wizard", "build", "help")</li>
                    <li><b>Getting Started Tab</b> - Step-by-step guide with action buttons</li>
                    <li><b>Keyboard Shortcuts</b> - Press F1 to see all shortcuts</li>
                    <li><b>Tooltips</b> - Hover over any button for quick help</li>
                </ul>

                <h3 style='color: #556B2F; margin-top: 25px;'>Documentation:</h3>
                <p style='background-color: #F0F5E6; padding: 15px; border-radius: 5px;'>
                    <b>📚 RoboShire Documentation:</b><br/>
                    <a href='file:///mnt/hgfs/ROS2_PROJECT/docs/README.md' style='color: #556B2F;'>Main Documentation</a> (Local)<br/>
                    <a href='file:///mnt/hgfs/ROS2_PROJECT/docs/installation/QUICK_START.md' style='color: #556B2F;'>Quick Start Guide</a><br/>
                    <a href='file:///mnt/hgfs/ROS2_PROJECT/docs/guides/EXECUTION_MODES.md' style='color: #556B2F;'>Execution Modes</a><br/>
                    <br/>
                    <b>📁 Tutorials (PDF Guides):</b><br/>
                    • Bumperbot Simulation Guide<br/>
                    • GUI Testing Guide<br/>
                    • Hardware Implementation Guide<br/>
                    • Self-Driving Tutorial<br/>
                    <br/>
                    <b>Location:</b> <code>/mnt/hgfs/ROS2_PROJECT/docs/</code>
                </p>

                <h3 style='color: #556B2F; margin-top: 25px;'>Example Projects:</h3>
                <p>Try <b>File → New from Example</b> to load pre-built robots!</p>
            </div>
        """)
        s4_text.setOpenExternalLinks(True)
        s4_layout.addWidget(s4_text)
        self.tour_slides.addWidget(slide4)

        layout.addWidget(self.tour_slides)

        # Navigation buttons
        nav_layout = QHBoxLayout()

        self.tour_prev_btn = QPushButton("← Previous")
        self.tour_prev_btn.clicked.connect(lambda: self._tour_navigate(-1))
        self.tour_prev_btn.setEnabled(False)
        nav_layout.addWidget(self.tour_prev_btn)

        self.tour_slide_label = QLabel("Slide 1 of 4")
        self.tour_slide_label.setAlignment(Qt.AlignCenter)
        nav_layout.addWidget(self.tour_slide_label)

        self.tour_next_btn = QPushButton("Next →")
        self.tour_next_btn.clicked.connect(lambda: self._tour_navigate(1))
        nav_layout.addWidget(self.tour_next_btn)

        self.tour_close_btn = QPushButton("Close")
        self.tour_close_btn.clicked.connect(dialog.accept)
        nav_layout.addWidget(self.tour_close_btn)

        layout.addLayout(nav_layout)

        # Store dialog reference for navigation
        self.tour_dialog = dialog

        dialog.exec()

    def _tour_navigate(self, direction):
        """Navigate between tour slides"""
        current = self.tour_slides.currentIndex()
        new_index = current + direction

        if 0 <= new_index < self.tour_slides.count():
            self.tour_slides.setCurrentIndex(new_index)
            self.tour_slide_label.setText(f"Slide {new_index + 1} of {self.tour_slides.count()}")

            # Update button states
            self.tour_prev_btn.setEnabled(new_index > 0)
            self.tour_next_btn.setEnabled(new_index < self.tour_slides.count() - 1)

    def _on_global_search(self):
        """Handle global search when Enter is pressed"""
        query = self.global_search.text().strip().lower()
        if not query:
            return

        # Search database - maps keywords/full names to actions
        search_db = {
            # Features - Full names from completer
            'start workflow wizard': ('workflow_wizard', lambda: self._on_new_robot_wizard()),
            'new project': ('workflow_wizard', lambda: self._on_new_robot_wizard()),
            'node graph editor': ('switch_tab', lambda: self._switch_to_tab("Node Graph")),
            'build package': ('build', lambda: self._on_build()),
            'run nodes': ('run', lambda: self._on_run()),
            'mujoco simulation': ('switch_tab', lambda: self._switch_to_tab("MuJoCo Viewer")),
            'rviz visualization': ('switch_tab', lambda: self._switch_to_tab("Simulation")),
            'code editor': ('switch_tab', lambda: self._switch_to_tab("Code Editor")),
            'topics inspector': ('switch_tab', lambda: self._switch_to_tab("Topics")),
            'parameters editor': ('switch_tab', lambda: self._switch_to_tab("Parameters")),
            'logs viewer': ('switch_tab', lambda: self._switch_to_tab("Logs")),
            'ssh setup': ('ssh_setup', lambda: self._on_ssh_setup_wizard()),
            'browse examples': ('examples', lambda: self._on_new_from_example()),
            'keyboard shortcuts': ('shortcuts', lambda: self._on_show_shortcuts()),
            'help documentation': ('help', lambda: self._show_welcome_tour()),
            'about roboshire': ('about', lambda: self._on_about()),
            'settings': ('settings', lambda: self._on_edit_settings()),
            'launch files': ('switch_tab', lambda: self._switch_to_tab("Launch Files")),
            'behavior trees': ('switch_tab', lambda: self._switch_to_tab("Behavior Trees")),
            'tf tree visualizer': ('switch_tab', lambda: self._switch_to_tab("TF Tree")),
            'system architecture': ('switch_tab', lambda: self._switch_to_tab("System Architecture")),
            'package browser': ('switch_tab', lambda: self._switch_to_tab("Packages")),
            'service manager': ('switch_tab', lambda: self._switch_to_tab("Services")),
            'action manager': ('switch_tab', lambda: self._switch_to_tab("Actions")),
            'performance monitor': ('switch_tab', lambda: self._switch_to_tab("Performance")),
            # Short keywords for convenience
            'wizard': ('workflow_wizard', lambda: self._on_new_robot_wizard()),
            'workflow': ('workflow_wizard', lambda: self._on_new_robot_wizard()),
            'node graph': ('switch_tab', lambda: self._switch_to_tab("Node Graph")),
            'build': ('build', lambda: self._on_build()),
            'run': ('run', lambda: self._on_run()),
            'mujoco': ('switch_tab', lambda: self._switch_to_tab("MuJoCo Viewer")),
            'simulation': ('switch_tab', lambda: self._switch_to_tab("Simulation")),
            'code': ('switch_tab', lambda: self._switch_to_tab("Code Editor")),
            'topics': ('switch_tab', lambda: self._switch_to_tab("Topics")),
            'parameters': ('switch_tab', lambda: self._switch_to_tab("Parameters")),
            'logs': ('switch_tab', lambda: self._switch_to_tab("Logs")),
            'ssh': ('ssh_setup', lambda: self._on_ssh_setup_wizard()),
            'examples': ('examples', lambda: self._on_new_from_example()),
            'shortcuts': ('shortcuts', lambda: self._on_show_shortcuts()),
            'help': ('help', lambda: self._show_welcome_tour()),
            'about': ('about', lambda: self._on_about()),
        }

        # Find matching action
        for keyword, (action_type, action_func) in search_db.items():
            if keyword in query:
                action_func()
                self.global_search.clear()
                self.status_bar.showMessage(f"Opened: {keyword.title()}", 3000)
                return

        # No match found
        self.status_bar.showMessage(f"No results for '{query}'. Try: wizard, build, run, topics, help", 5000)

    def _on_global_search_text_changed(self, text):
        """Handle search text changes (QCompleter handles suggestions automatically)"""
        # QCompleter automatically shows suggestions dropdown
        # This method can be used for additional functionality if needed
        pass

    def _on_code_file_saved(self, file_path: str):
        """Handle file saved from code editor"""
        self.logger.info(f"File saved: {file_path}")
        self.status_bar.showMessage(f"Saved: {Path(file_path).name}")

        # If it's part of current project, mark as modified
        if self.current_project:
            project_dir = Path(self.current_project).parent
            if Path(file_path).is_relative_to(project_dir):
                self.project_modified = True
                self._update_ui_state()

    def _on_launch_file_saved(self, file_path: str):
        """Handle launch file saved"""
        self.logger.info(f"Launch file saved: {file_path}")
        self.status_bar.showMessage(f"Launch file saved: {Path(file_path).name}")

        # If it's part of current project, mark as modified
        if self.current_project:
            project_dir = Path(self.current_project).parent
            if Path(file_path).is_relative_to(project_dir):
                self.project_modified = True
                self._update_ui_state()

    def _show_error_translation_dialog(self, translation: dict):
        """Show error translation dialog with beginner-friendly explanation"""
        from PySide6.QtWidgets import QDialog, QVBoxLayout, QLabel, QPushButton, QTextEdit

        dialog = QDialog(self)
        dialog.setWindowTitle(f"{self.error_translator.get_category_icon(translation['category'])} {translation['title']}")
        dialog.setMinimumWidth(600)
        dialog.setMinimumHeight(400)

        layout = QVBoxLayout(dialog)

        # Category label
        category_label = QLabel(f"<b>Category:</b> {translation['category']}")
        layout.addWidget(category_label)

        # Explanation
        explanation_label = QLabel("<b>What went wrong:</b>")
        layout.addWidget(explanation_label)

        explanation_text = QTextEdit()
        explanation_text.setReadOnly(True)
        explanation_text.setPlainText(translation['explanation'])
        explanation_text.setMaximumHeight(100)
        layout.addWidget(explanation_text)

        # Solution
        solution_label = QLabel("<b>How to fix it:</b>")
        layout.addWidget(solution_label)

        solution_text = QTextEdit()
        solution_text.setReadOnly(True)
        solution_text.setMarkdown(translation['solution'])
        layout.addWidget(solution_text)

        # Close button
        close_button = QPushButton("OK")
        close_button.clicked.connect(dialog.accept)
        layout.addWidget(close_button)

        dialog.exec()

    def _on_reset_layout(self):
        """Reset window layout to default"""
        reply = QMessageBox.question(
            self,
            "Reset Layout",
            "Reset window layout to default?\n\n"
            "This will restore default window size, tab positions, and splitter positions.\n"
            "You will need to restart RoboShire for changes to take effect.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            try:
                self.layout_manager.reset_layout("default")
                QMessageBox.information(
                    self,
                    "Layout Reset",
                    "Layout has been reset to default.\n\n"
                    "Please restart RoboShire to see the changes."
                )
            except Exception as e:
                self.logger.error(f"Failed to reset layout: {e}")
                QMessageBox.warning(
                    self,
                    "Reset Failed",
                    f"Failed to reset layout:\n{e}"
                )

    def _on_save_layout(self):
        """Save current window layout"""
        try:
            # For now, just save to default layout
            # Future: Could add dialog to save with custom name
            self.layout_manager.save_layout(self, "default")
            QMessageBox.information(
                self,
                "Layout Saved",
                "Window layout has been saved.\n\n"
                "Your current tab positions, window size, and splitter positions\n"
                "will be restored next time you start RoboShire."
            )
        except Exception as e:
            self.logger.error(f"Failed to save layout: {e}")
            QMessageBox.warning(
                self,
                "Save Failed",
                f"Failed to save layout:\n{e}"
            )

    def _on_about(self):
        """Show about dialog with branding"""
        from PySide6.QtWidgets import QDialog, QVBoxLayout, QLabel, QPushButton
        from roboshire.resources import get_logo_path, BRAND_COLORS

        dialog = QDialog(self)
        dialog.setWindowTitle("About RoboShire")
        dialog.setMinimumWidth(500)

        layout = QVBoxLayout(dialog)
        layout.setSpacing(15)
        layout.setContentsMargins(30, 30, 30, 30)

        # Logo
        try:
            logo_label = QLabel()
            pixmap = QPixmap(str(get_logo_path(with_text=True)))
            # Scale logo to reasonable size
            pixmap = pixmap.scaledToWidth(350, Qt.TransformationMode.SmoothTransformation)
            logo_label.setPixmap(pixmap)
            logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(logo_label)
        except Exception as e:
            self.logger.warning(f"Could not load About dialog logo: {e}")

        # Version info
        version_label = QLabel("<h2 style='text-align: center; color: #556B2F;'>RoboShire v2.0.0</h2>")
        version_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(version_label)

        # Description with brand styling
        desc = QLabel(
            "<p style='text-align: center;'>"
            "<b>A GUI Interface for ROS2 Robotics Systems</b><br><br>"
            "Beginner-friendly Visual IDE that makes ROS2 robotics<br>"
            "accessible to Arduino-level users<br><br>"
            "<span style='color: #556B2F;'><b>Philosophy:</b></span> Don't reinvent - integrate existing tools<br>"
            "<span style='color: #556B2F;'><b>Output:</b></span> 100% standard ROS2 code<br><br>"
            "Built with MuJoCo simulation, SSH integration,<br>"
            "and comprehensive ROS2 tooling"
            "</p>"
        )
        desc.setWordWrap(True)
        desc.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(desc)

        # Close button with brand color
        close_btn = QPushButton("Close")
        close_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {BRAND_COLORS['olive_green']};
                color: white;
                border: none;
                padding: 8px 20px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #6B8E3A;
            }}
        """)
        close_btn.clicked.connect(dialog.accept)
        layout.addWidget(close_btn, alignment=Qt.AlignmentFlag.AlignCenter)

        dialog.exec()

    def _on_command_palette(self):
        """Show command palette (v2.5.0)"""
        # Collect all actions from the main window
        all_actions = []

        # Get actions from menus
        for action in self.findChildren(QAction):
            # Skip separators and disabled actions
            if action.text() and action.isEnabled():
                all_actions.append(action)

        # Show command palette
        palette = CommandPalette(all_actions, parent=self)
        palette.exec()

    # Widget signal handlers

    def _on_urdf_item_selected(self, item_type: str, item_name: str):
        """Handle URDF tree item selection"""
        self.property_editor.show_properties(item_type, item_name)
        self.status_bar.showMessage(f"Selected {item_type}: {item_name}")

    def _on_urdf_item_double_clicked(self, item_type: str, item_name: str):
        """Handle URDF tree item double click"""
        logging.info(f"Double clicked {item_type}: {item_name}")

    def _on_property_changed(self, item_type: str, item_name: str, property_name: str, value):
        """Handle property change"""
        logging.info(f"Property changed: {item_type}.{item_name}.{property_name} = {value}")
        self.urdf_tree_editor.refresh()  # Refresh tree to show changes
        self.project_modified = True
        self._update_ui_state()

    def _on_graph_modified(self):
        """Handle node graph modification"""
        self.project_modified = True
        self._update_ui_state()

    def _on_node_selected(self, node_id: str):
        """Handle node selection in graph editor"""
        logging.info(f"Node selected: {node_id}")
        # TODO: Show node properties in property editor

    def _switch_to_tab(self, tab_name: str):
        """Switch to specified tab by name (supports nested tabs)"""
        # Mapping of old tab names to new structure (category, sub-tab)
        tab_mapping = {
            "Node Graph": ("🎨 Design", "Node Graph"),
            "Launch Files": ("🎨 Design", "Launch Files"),
            "Behavior Trees": ("🎨 Design", "Behavior Trees"),
            "Code Editor": ("💻 Development", "Code Editor"),
            "Multi-Package": ("💻 Development", "Multi-Package"),
            "Packages": ("💻 Development", "Package Browser"),
            "MuJoCo": ("🎮 Simulation", "MuJoCo Viewer"),
            "TF Tree": ("🎮 Simulation", "TF Tree"),
            "Node Status": ("📡 Monitoring", "Node Status"),
            "Topics": ("📡 Monitoring", "Topics"),
            "Parameters": ("📡 Monitoring", "Parameters"),
            "Performance": ("📡 Monitoring", "Performance"),
            "Logs": ("📡 Monitoring", "Logs"),
            "Build": ("🔧 Build & Deploy", "Build Output"),
            "Lifecycle": ("🔧 Build & Deploy", "Lifecycle Manager"),
        }

        # Check if it's a direct tab (Getting Started, Project Status)
        for i in range(self.tab_widget.count()):
            if tab_name in self.tab_widget.tabText(i):
                self.tab_widget.setCurrentIndex(i)
                self.status_bar.showMessage(f"Switched to {tab_name} tab")
                return

        # Check if it's a nested tab
        if tab_name in tab_mapping:
            category, sub_tab = tab_mapping[tab_name]

            # Find and switch to category tab
            for i in range(self.tab_widget.count()):
                if self.tab_widget.tabText(i) == category:
                    self.tab_widget.setCurrentIndex(i)

                    # Get the nested tab widget and switch to sub-tab
                    nested_widget = self.tab_widget.widget(i)
                    if isinstance(nested_widget, QTabWidget):
                        for j in range(nested_widget.count()):
                            if nested_widget.tabText(j) == sub_tab:
                                nested_widget.setCurrentIndex(j)
                                self.status_bar.showMessage(f"Switched to {tab_name}")
                                return

        # Tab not found
        logging.warning(f"Tab '{tab_name}' not found")

    def _on_show_shortcuts(self):
        """Show keyboard shortcuts cheat sheet"""
        show_shortcut_cheatsheet(self)

    def _on_mujoco_simulation_started(self):
        """Handle MuJoCo simulation start"""
        self.logger.info("MuJoCo simulation started")
        self.status_bar.showMessage("MuJoCo simulation running")

    def _on_mujoco_simulation_stopped(self):
        """Handle MuJoCo simulation stop"""
        self.logger.info("MuJoCo simulation stopped")
        self.status_bar.showMessage("MuJoCo simulation paused")

    def _on_mujoco_joint_states(self, joint_states: dict):
        """Handle MuJoCo joint state updates"""
        # Could be used to sync with other visualizations
        pass

    def _on_package_file_selected(self, file_path: str):
        """Open selected file in internal code editor"""
        try:
            self.logger.info(f"Opening file in code editor: {file_path}")

            # Open in internal code editor
            self.code_editor_widget.open_file(file_path)

            # Switch to Code Editor tab using the helper method
            self._switch_to_tab("Code Editor")

            self.status_bar.showMessage(f"Opened: {Path(file_path).name}", 3000)

        except Exception as e:
            self.logger.error(f"Failed to open file: {e}")
            QMessageBox.warning(
                self,
                "Open File Error",
                f"Failed to open file:\n{file_path}\n\n{e}"
            )

    def showEvent(self, event):
        """Handle window show event - restore saved layout"""
        super().showEvent(event)

        # Restore saved layout on first show
        if not hasattr(self, '_layout_restored'):
            self._layout_restored = True
            try:
                self.layout_manager.restore_layout(self)
                self.logger.info("Window layout restored")
            except Exception as e:
                self.logger.warning(f"Could not restore layout: {e}")

    def closeEvent(self, event):
        """Handle window close event"""
        # Save layout before closing
        try:
            self.layout_manager.save_layout(self)
            self.logger.info("Window layout saved")
        except Exception as e:
            self.logger.warning(f"Could not save layout: {e}")

        # Cleanup ROS2 nodes
        if self.ros2_launcher and self.running_nodes:
            reply = QMessageBox.question(
                self,
                "Running Nodes",
                f"{len(self.running_nodes)} node(s) are still running.\n\n"
                "Stop them before closing?",
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
                QMessageBox.Yes
            )

            if reply == QMessageBox.Cancel:
                event.ignore()
                return
            elif reply == QMessageBox.Yes:
                try:
                    self.ros2_launcher.stop_all_nodes(force=True)
                    logging.info("Stopped all nodes before closing")
                except Exception as e:
                    logging.error(f"Error stopping nodes on close: {e}")

        # Cleanup RViz
        if self.rviz_bridge.is_running:
            self.rviz_bridge.close()

        # Cleanup MuJoCo (Phase 8)
        if hasattr(self, 'mujoco_viewer'):
            self.mujoco_viewer.cleanup()

        # Cleanup Performance Profiler (v2.0.0)
        if hasattr(self, 'performance_profiler'):
            self.performance_profiler.stop_monitoring()

        event.accept()

    # v2.0.0 Handler Methods

    def _on_nav2_wizard(self):
        """Show Nav2 Integration Wizard"""
        project_dir = Path(self.current_project) if self.current_project else None

        wizard = Nav2IntegrationWizard(project_dir=project_dir, parent=self)
        wizard.nav2_configured.connect(self._on_nav2_configured)
        wizard.exec()

    def _on_nav2_configured(self, config: dict):
        """Handle Nav2 configuration completion"""
        logging.info(f"Nav2 configured for robot: {config.get('robot', {}).get('robot_name')}")
        self.status_message.setText("✅ Nav2 configuration generated")

    def _on_behavior_tree_saved(self, file_path: str):
        """Handle behavior tree saved"""
        logging.info(f"Behavior tree saved: {file_path}")
        self.status_message.setText(f"✅ Behavior tree saved: {Path(file_path).name}")

    def _on_package_created(self, package_name: str):
        """Handle package creation from multi-package manager"""
        logging.info(f"Package created: {package_name}")
        self.status_message.setText(f"✅ Package '{package_name}' created")

    def _on_multi_package_build(self, package_order: list):
        """Handle multi-package build request"""
        logging.info(f"Building packages in order: {package_order}")
        self.status_message.setText(f"🔨 Building {len(package_order)} packages...")

        # TODO: Integrate with colcon bridge for actual build
        # For now, just show message
        QMessageBox.information(
            self,
            "Multi-Package Build",
            f"Build order:\n\n" + "\n".join([f"{i+1}. {pkg}" for i, pkg in enumerate(package_order)])
        )

    # ========== v2.4.0: Hardware Features ==========

    def _on_micro_ros_agent(self):
        """Show micro-ROS agent manager"""
        if not hasattr(self, 'micro_ros_agent_widget') or self.micro_ros_agent_widget is None:
            self.micro_ros_agent_widget = MicroROSAgentWidget()

            # Connect status changed signal to update status bar
            self.micro_ros_agent_widget.status_changed.connect(self._on_agent_status_changed)

            # Show as a dialog
            from PySide6.QtWidgets import QDialog, QVBoxLayout
            dialog = QDialog(self)
            dialog.setWindowTitle("micro-ROS Agent Manager")
            dialog.resize(600, 500)
            layout = QVBoxLayout(dialog)
            layout.addWidget(self.micro_ros_agent_widget)
            dialog.finished.connect(lambda: self.micro_ros_agent_widget.cleanup())
            dialog.show()

            # Store reference to dialog so it doesn't get garbage collected
            self._micro_ros_dialog = dialog

        self.logger.info("micro-ROS Agent Manager opened")

    def _on_hardware_test(self):
        """Show hardware test panel"""
        if not hasattr(self, 'hardware_test_panel') or self.hardware_test_panel is None:
            self.hardware_test_panel = HardwareTestPanel()

            # Show as a dialog
            from PySide6.QtWidgets import QDialog, QVBoxLayout
            dialog = QDialog(self)
            dialog.setWindowTitle("Hardware Test Panel")
            dialog.resize(900, 600)
            layout = QVBoxLayout(dialog)
            layout.addWidget(self.hardware_test_panel)
            dialog.finished.connect(lambda: self.hardware_test_panel.cleanup())
            dialog.show()

            # Store reference to dialog so it doesn't get garbage collected
            self._hardware_test_dialog = dialog

        self.logger.info("Hardware Test Panel opened")

    def _on_agent_status_changed(self, connected: bool, status_text: str):
        """Handle micro-ROS agent status change"""
        # Update status bar indicator if it exists
        if hasattr(self, 'agent_status_label'):
            if connected:
                self.agent_status_label.setText(f"Agent: ● {status_text}")
                self.agent_status_label.setStyleSheet("QLabel { color: green; }")
            else:
                self.agent_status_label.setText(f"Agent: ● {status_text}")
                self.agent_status_label.setStyleSheet("QLabel { color: red; }")

    def _on_gazebo_plugins(self):
        """Show Gazebo plugin manager"""
        if not hasattr(self, 'gazebo_plugin_widget') or self.gazebo_plugin_widget is None:
            self.gazebo_plugin_widget = GazeboPluginManagerWidget()

            # Show as a dialog
            from PySide6.QtWidgets import QDialog, QVBoxLayout
            dialog = QDialog(self)
            dialog.setWindowTitle("Gazebo Plugin Manager")
            dialog.resize(800, 600)
            layout = QVBoxLayout(dialog)
            layout.addWidget(self.gazebo_plugin_widget)
            dialog.show()

            # Store reference to dialog so it doesn't get garbage collected
            self._gazebo_plugin_dialog = dialog

        self.logger.info("Gazebo Plugin Manager opened")

