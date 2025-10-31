"""
Multi-Package Manager - GUI for managing multiple ROS2 packages in workspace

This widget provides package creation, dependency management, and workspace-level
build coordination for multi-package projects.

Author: RoboShire Team
Version: 2.0.1 (v2.0.1 Critical Fix: Circular Dependency Detection)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTreeWidget, QTreeWidgetItem,
    QPushButton, QLabel, QGroupBox, QSplitter, QTextEdit, QInputDialog,
    QMessageBox, QMenu, QHeaderView, QCheckBox, QFormLayout, QLineEdit,
    QComboBox, QListWidget
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QColor, QAction
from pathlib import Path
from typing import Dict, List, Optional, Set
import logging


class MultiPackageManager(QWidget):
    """
    Multi-Package Workspace Manager

    Features:
    - Create multiple ROS2 packages in workspace
    - Manage inter-package dependencies
    - Visualize dependency graph
    - Workspace-level build coordination
    - Package dependency validation
    """

    package_created = Signal(str)
    package_removed = Signal(str)
    build_requested = Signal(list)  # List of package names

    def __init__(self, workspace_path: Optional[Path] = None, parent=None):
        super().__init__(parent)

        self.workspace_path = workspace_path
        self.packages: Dict[str, Dict] = {}  # package_name -> package_info

        self._init_ui()

    def _init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)

        # Title and controls
        header_layout = QHBoxLayout()
        header_layout.addWidget(QLabel("<h3>Multi-Package Workspace</h3>"))
        header_layout.addStretch()

        # Workspace path
        if self.workspace_path:
            path_label = QLabel(f"Workspace: {self.workspace_path}")
            path_label.setStyleSheet("color: #666;")
            header_layout.addWidget(path_label)

        layout.addLayout(header_layout)

        # Action buttons
        actions_layout = QHBoxLayout()

        create_pkg_btn = QPushButton("Create Package")
        create_pkg_btn.clicked.connect(self._create_package)
        actions_layout.addWidget(create_pkg_btn)

        refresh_btn = QPushButton("Refresh Packages")
        refresh_btn.clicked.connect(self._refresh_packages)
        actions_layout.addWidget(refresh_btn)

        validate_btn = QPushButton("Validate Dependencies")
        validate_btn.clicked.connect(self._validate_dependencies)
        actions_layout.addWidget(validate_btn)

        actions_layout.addStretch()

        build_all_btn = QPushButton("Build All Packages")
        build_all_btn.clicked.connect(self._build_all)
        actions_layout.addWidget(build_all_btn)

        build_selected_btn = QPushButton("Build Selected")
        build_selected_btn.clicked.connect(self._build_selected)
        actions_layout.addWidget(build_selected_btn)

        layout.addLayout(actions_layout)

        # Main content - splitter
        splitter = QSplitter(Qt.Horizontal)

        # Left panel - Package tree
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        left_layout.addWidget(QLabel("<b>Packages in Workspace</b>"))

        self.package_tree = QTreeWidget()
        self.package_tree.setHeaderLabels(["Package", "Type", "Dependencies"])
        self.package_tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.package_tree.customContextMenuRequested.connect(self._show_context_menu)
        self.package_tree.itemSelectionChanged.connect(self._on_package_selected)
        left_layout.addWidget(self.package_tree)

        # Statistics
        stats_group = QGroupBox("Statistics")
        stats_layout = QVBoxLayout(stats_group)

        self.stats_label = QLabel("Total Packages: 0\nPython Packages: 0\nC++ Packages: 0")
        stats_layout.addWidget(self.stats_label)

        left_layout.addWidget(stats_group)

        # Right panel - Package details and dependency graph
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        # Package details
        details_group = QGroupBox("Package Details")
        details_layout = QFormLayout(details_group)

        self.name_label = QLabel("-")
        details_layout.addRow("Name:", self.name_label)

        self.type_label = QLabel("-")
        details_layout.addRow("Type:", self.type_label)

        self.path_label = QLabel("-")
        self.path_label.setWordWrap(True)
        details_layout.addRow("Path:", self.path_label)

        self.deps_label = QLabel("-")
        self.deps_label.setWordWrap(True)
        details_layout.addRow("Dependencies:", self.deps_label)

        right_layout.addWidget(details_group)

        # Dependency graph (text visualization)
        graph_group = QGroupBox("Dependency Graph")
        graph_layout = QVBoxLayout(graph_group)

        self.graph_text = QTextEdit()
        self.graph_text.setReadOnly(True)
        self.graph_text.setFont(QFont("Consolas", 9))
        graph_layout.addWidget(self.graph_text)

        right_layout.addWidget(graph_group)

        # Build order
        order_group = QGroupBox("Build Order")
        order_layout = QVBoxLayout(order_group)

        self.build_order_list = QListWidget()
        order_layout.addWidget(self.build_order_list)

        right_layout.addWidget(order_group)

        # Add panels to splitter
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter)

        # Status bar
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("color: #666; padding: 5px;")
        layout.addWidget(self.status_label)

    def _create_package(self):
        """Show create package dialog"""
        from PySide6.QtWidgets import QDialog, QDialogButtonBox

        dialog = QDialog(self)
        dialog.setWindowTitle("Create New Package")
        dialog.setMinimumWidth(400)

        layout = QFormLayout(dialog)

        # Package name
        name_edit = QLineEdit()
        layout.addRow("Package Name:", name_edit)

        # Package type
        type_combo = QComboBox()
        type_combo.addItems(["ament_python", "ament_cmake"])
        layout.addRow("Package Type:", type_combo)

        # Dependencies
        deps_edit = QLineEdit()
        deps_edit.setPlaceholderText("rclpy, std_msgs, geometry_msgs")
        layout.addRow("Dependencies (comma-separated):", deps_edit)

        # Description
        desc_edit = QLineEdit()
        desc_edit.setPlaceholderText("Package description")
        layout.addRow("Description:", desc_edit)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)

        if dialog.exec() == QDialog.Accepted:
            pkg_name = name_edit.text().strip()

            if not pkg_name:
                QMessageBox.warning(self, "Invalid Name", "Package name cannot be empty")
                return

            # Parse dependencies
            deps_text = deps_edit.text().strip()
            dependencies = [d.strip() for d in deps_text.split(',') if d.strip()]

            # Create package info
            package_info = {
                'name': pkg_name,
                'type': type_combo.currentText(),
                'dependencies': dependencies,
                'description': desc_edit.text().strip()
            }

            # Add to packages dict
            self.packages[pkg_name] = package_info

            # Update UI
            self._refresh_package_tree()
            self._update_dependency_graph()
            self._update_build_order()

            self.status_label.setText(f"✅ Package '{pkg_name}' created")
            self.package_created.emit(pkg_name)

    def _refresh_packages(self):
        """Refresh package list from workspace"""
        if not self.workspace_path:
            self.status_label.setText("⚠️ No workspace path set")
            return

        # In production, scan workspace/src for packages
        # For now, just update the tree
        self._refresh_package_tree()
        self.status_label.setText("✅ Packages refreshed")

    def _refresh_package_tree(self):
        """Refresh the package tree widget"""
        self.package_tree.clear()

        python_count = 0
        cpp_count = 0

        for pkg_name, pkg_info in self.packages.items():
            item = QTreeWidgetItem([
                pkg_name,
                pkg_info['type'],
                ', '.join(pkg_info['dependencies'][:3]) + ('...' if len(pkg_info['dependencies']) > 3 else '')
            ])

            # Color code by type
            if pkg_info['type'] == 'ament_python':
                item.setForeground(0, QColor(100, 150, 255))
                python_count += 1
            else:
                item.setForeground(0, QColor(100, 200, 100))
                cpp_count += 1

            self.package_tree.addTopLevelItem(item)

        # Update statistics
        total = len(self.packages)
        self.stats_label.setText(
            f"Total Packages: {total}\n"
            f"Python Packages: {python_count}\n"
            f"C++ Packages: {cpp_count}"
        )

        self.package_tree.expandAll()

    def _on_package_selected(self):
        """Handle package selection"""
        selected_items = self.package_tree.selectedItems()

        if not selected_items:
            return

        pkg_name = selected_items[0].text(0)
        pkg_info = self.packages.get(pkg_name)

        if pkg_info:
            self.name_label.setText(pkg_name)
            self.type_label.setText(pkg_info['type'])
            self.path_label.setText(str(self.workspace_path / "src" / pkg_name) if self.workspace_path else "-")
            self.deps_label.setText(', '.join(pkg_info['dependencies']) if pkg_info['dependencies'] else "None")

    def _show_context_menu(self, position):
        """Show context menu for package"""
        item = self.package_tree.itemAt(position)

        if not item:
            return

        pkg_name = item.text(0)

        menu = QMenu(self)

        add_dep_action = QAction("Add Dependency", self)
        add_dep_action.triggered.connect(lambda: self._add_dependency(pkg_name))
        menu.addAction(add_dep_action)

        remove_dep_action = QAction("Remove Dependency", self)
        remove_dep_action.triggered.connect(lambda: self._remove_dependency(pkg_name))
        menu.addAction(remove_dep_action)

        menu.addSeparator()

        delete_action = QAction("Delete Package", self)
        delete_action.triggered.connect(lambda: self._delete_package(pkg_name))
        menu.addAction(delete_action)

        menu.exec(self.package_tree.viewport().mapToGlobal(position))

    def _add_dependency(self, pkg_name: str):
        """Add dependency to package"""
        dep_name, ok = QInputDialog.getText(
            self,
            "Add Dependency",
            f"Enter dependency name for '{pkg_name}':"
        )

        if ok and dep_name:
            if pkg_name in self.packages:
                if dep_name not in self.packages[pkg_name]['dependencies']:
                    self.packages[pkg_name]['dependencies'].append(dep_name)
                    self._refresh_package_tree()
                    self._update_dependency_graph()
                    self._update_build_order()
                    self.status_label.setText(f"✅ Added dependency '{dep_name}' to '{pkg_name}'")

    def _remove_dependency(self, pkg_name: str):
        """Remove dependency from package"""
        if pkg_name not in self.packages or not self.packages[pkg_name]['dependencies']:
            QMessageBox.information(self, "No Dependencies", f"Package '{pkg_name}' has no dependencies")
            return

        dep_name, ok = QInputDialog.getItem(
            self,
            "Remove Dependency",
            f"Select dependency to remove from '{pkg_name}':",
            self.packages[pkg_name]['dependencies'],
            0,
            False
        )

        if ok and dep_name:
            self.packages[pkg_name]['dependencies'].remove(dep_name)
            self._refresh_package_tree()
            self._update_dependency_graph()
            self._update_build_order()
            self.status_label.setText(f"✅ Removed dependency '{dep_name}' from '{pkg_name}'")

    def _delete_package(self, pkg_name: str):
        """Delete a package"""
        reply = QMessageBox.question(
            self,
            "Delete Package",
            f"Are you sure you want to delete package '{pkg_name}'?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            del self.packages[pkg_name]
            self._refresh_package_tree()
            self._update_dependency_graph()
            self._update_build_order()
            self.status_label.setText(f"✅ Deleted package '{pkg_name}'")
            self.package_removed.emit(pkg_name)

    def _validate_dependencies(self):
        """v2.0.1: Validate all package dependencies including circular detection"""
        issues = []

        # Check for missing dependencies
        for pkg_name, pkg_info in self.packages.items():
            for dep in pkg_info['dependencies']:
                # Check if dependency is in workspace
                if dep not in self.packages and not self._is_system_package(dep):
                    issues.append(f"Missing: {pkg_name} → {dep}")

        # v2.0.1: Check for circular dependencies
        cycles = self._detect_circular_dependencies()

        if cycles:
            for cycle in cycles:
                cycle_str = " → ".join(cycle)
                issues.append(f"Circular: {cycle_str}")

        if issues:
            # Separate missing and circular issues
            missing_issues = [i for i in issues if i.startswith("Missing:")]
            circular_issues = [i for i in issues if i.startswith("Circular:")]

            msg_parts = []
            if missing_issues:
                msg_parts.append(f"Missing Dependencies ({len(missing_issues)}):\n" + "\n".join(missing_issues[:5]))
                if len(missing_issues) > 5:
                    msg_parts[-1] += f"\n... and {len(missing_issues) - 5} more"

            if circular_issues:
                msg_parts.append(f"\nCircular Dependencies ({len(circular_issues)}):\n" + "\n".join(circular_issues[:5]))
                if len(circular_issues) > 5:
                    msg_parts[-1] += f"\n... and {len(circular_issues) - 5} more"

            QMessageBox.warning(
                self,
                "Dependency Issues",
                f"Found {len(issues)} dependency issues:\n\n" + "\n".join(msg_parts)
            )
            self.status_label.setText(f"⚠️ {len(issues)} dependency issues found ({len(circular_issues)} circular)")
        else:
            QMessageBox.information(
                self,
                "Validation Success",
                "✅ All dependencies are valid! No circular dependencies detected."
            )
            self.status_label.setText("✅ All dependencies valid, no cycles")

    def _detect_circular_dependencies(self) -> List[List[str]]:
        """
        v2.0.1: Detect all circular dependencies using Depth-First Search

        Returns:
            List of cycles, where each cycle is a list of package names forming a circle
        """
        cycles = []
        visited = set()
        rec_stack = set()

        def dfs(pkg_name: str, path: List[str]) -> bool:
            """DFS helper to detect cycles"""
            if pkg_name not in self.packages:
                # External/system package, no cycle
                return False

            if pkg_name in rec_stack:
                # Found a cycle! Extract it from path
                cycle_start_idx = path.index(pkg_name)
                cycle = path[cycle_start_idx:] + [pkg_name]

                # Check if this cycle is already recorded (in any rotation)
                cycle_set = set(cycle[:-1])  # Remove duplicate end element
                is_new_cycle = True

                for existing_cycle in cycles:
                    if set(existing_cycle[:-1]) == cycle_set:
                        is_new_cycle = False
                        break

                if is_new_cycle:
                    cycles.append(cycle)

                return True

            if pkg_name in visited:
                return False

            # Mark as visiting
            rec_stack.add(pkg_name)
            path.append(pkg_name)

            # Visit all dependencies
            for dep in self.packages[pkg_name]['dependencies']:
                dfs(dep, path.copy())

            # Mark as visited
            path.pop()
            rec_stack.remove(pkg_name)
            visited.add(pkg_name)

            return False

        # Run DFS from each package
        for pkg_name in self.packages.keys():
            if pkg_name not in visited:
                dfs(pkg_name, [])

        return cycles

    def _is_system_package(self, pkg_name: str) -> bool:
        """Check if package is a system/ROS2 package"""
        system_packages = {
            'rclcpp', 'rclpy', 'std_msgs', 'sensor_msgs', 'geometry_msgs',
            'nav_msgs', 'tf2', 'tf2_ros', 'ament_cmake', 'ament_python'
        }
        return pkg_name in system_packages

    def _update_dependency_graph(self):
        """v2.0.1: Update dependency graph visualization with cycle detection"""
        lines = ["Dependency Graph:", ""]

        # v2.0.1: Detect cycles first
        cycles = self._detect_circular_dependencies()
        cycle_packages = set()

        for cycle in cycles:
            for pkg in cycle[:-1]:  # Exclude duplicate end element
                cycle_packages.add(pkg)

        # Build graph text
        for pkg_name, pkg_info in self.packages.items():
            # v2.0.1: Mark packages involved in cycles
            if pkg_name in cycle_packages:
                lines.append(f"{pkg_name} ⚠️ [IN CYCLE]")
            else:
                lines.append(f"{pkg_name}")

            for dep in pkg_info['dependencies']:
                is_workspace = dep in self.packages
                symbol = "├─" if is_workspace else "└─"
                marker = "[workspace]" if is_workspace else "[system]"

                # v2.0.1: Check if this edge is part of a cycle
                is_cycle_edge = False
                if pkg_name in cycle_packages and dep in cycle_packages:
                    # Check if this specific edge is part of any cycle
                    for cycle in cycles:
                        if pkg_name in cycle and dep in cycle:
                            pkg_idx = cycle.index(pkg_name)
                            if pkg_idx + 1 < len(cycle) and cycle[pkg_idx + 1] == dep:
                                is_cycle_edge = True
                                break

                if is_cycle_edge:
                    lines.append(f"  {symbol} {dep} {marker} ⚠️ CIRCULAR")
                else:
                    lines.append(f"  {symbol} {dep} {marker}")

            lines.append("")

        # v2.0.1: Add cycle summary at bottom
        if cycles:
            lines.append("=" * 50)
            lines.append(f"⚠️  CIRCULAR DEPENDENCIES DETECTED ({len(cycles)}):")
            lines.append("")

            for i, cycle in enumerate(cycles, 1):
                cycle_str = " → ".join(cycle)
                lines.append(f"  Cycle {i}: {cycle_str}")

        self.graph_text.setPlainText("\n".join(lines))

    def _update_build_order(self):
        """Update build order list"""
        self.build_order_list.clear()

        # Simple topological sort
        order = self._get_build_order()

        for i, pkg_name in enumerate(order):
            self.build_order_list.addItem(f"{i+1}. {pkg_name}")

    def _get_build_order(self) -> List[str]:
        """Get packages in dependency order"""
        visited = set()
        order = []

        def visit(pkg_name: str):
            if pkg_name in visited or pkg_name not in self.packages:
                return
            visited.add(pkg_name)

            # Visit dependencies first
            for dep in self.packages[pkg_name]['dependencies']:
                visit(dep)

            order.append(pkg_name)

        for pkg_name in self.packages:
            visit(pkg_name)

        return order

    def _build_all(self):
        """Build all packages in dependency order"""
        order = self._get_build_order()

        if not order:
            QMessageBox.information(self, "No Packages", "No packages to build")
            return

        self.status_label.setText(f"🔨 Building {len(order)} packages...")
        self.build_requested.emit(order)

    def _build_selected(self):
        """Build selected package and dependencies"""
        selected_items = self.package_tree.selectedItems()

        if not selected_items:
            QMessageBox.information(self, "No Selection", "Please select a package to build")
            return

        pkg_name = selected_items[0].text(0)

        # Get dependencies
        order = []
        visited = set()

        def visit(name: str):
            if name in visited or name not in self.packages:
                return
            visited.add(name)

            for dep in self.packages[name]['dependencies']:
                visit(dep)

            order.append(name)

        visit(pkg_name)

        self.status_label.setText(f"🔨 Building {pkg_name} and {len(order)-1} dependencies...")
        self.build_requested.emit(order)

    def set_workspace_path(self, path: Path):
        """Set workspace path"""
        self.workspace_path = path

    def get_packages(self) -> Dict[str, Dict]:
        """Get all packages"""
        return self.packages

    def add_package(self, pkg_info: Dict):
        """Add a package programmatically"""
        if 'name' in pkg_info:
            self.packages[pkg_info['name']] = pkg_info
            self._refresh_package_tree()
            self._update_dependency_graph()
            self._update_build_order()


# Standalone test
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    manager = MultiPackageManager(Path("/tmp/test_workspace"))
    manager.setWindowTitle("Multi-Package Manager")
    manager.resize(1200, 700)
    manager.show()

    # Add test packages
    manager.add_package({
        'name': 'robot_core',
        'type': 'ament_python',
        'dependencies': ['rclpy', 'std_msgs'],
        'description': 'Core robot package'
    })

    manager.add_package({
        'name': 'robot_navigation',
        'type': 'ament_python',
        'dependencies': ['rclpy', 'robot_core', 'geometry_msgs'],
        'description': 'Navigation package'
    })

    manager.add_package({
        'name': 'robot_perception',
        'type': 'ament_cmake',
        'dependencies': ['rclcpp', 'sensor_msgs', 'robot_core'],
        'description': 'Perception package'
    })

    sys.exit(app.exec())
