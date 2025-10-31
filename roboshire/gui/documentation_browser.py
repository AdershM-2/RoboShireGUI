"""
Documentation Browser

Integrated documentation search and viewer:
- Local documentation (markdown files in docs/)
- ROS2 API documentation with built-in entries
- Quick help tooltips
- Context-sensitive help

Author: RoboShire Team
Phase: 11 (Advanced Features)
Phase: 12 (Discovery & Search) - Week 2 Enhancement
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton,
    QListWidget, QListWidgetItem, QTextBrowser, QSplitter,
    QGroupBox, QLabel, QComboBox, QTabWidget, QMessageBox
)
from PySide6.QtCore import Qt, Signal, QUrl
from PySide6.QtGui import QFont
from pathlib import Path
from typing import List, Dict, Optional
import re
import webbrowser

from roboshire.backend.documentation_search import (
    DocumentationSearch, DocEntry, DocCategory
)


class DocumentationEntry:
    """Documentation entry"""
    def __init__(self, title: str, path: str, category: str = "general"):
        self.title = title
        self.path = path
        self.category = category
        self.content = ""
        self.keywords = []


class DocumentationBrowser(QWidget):
    """
    Integrated documentation browser with search
    """

    # Signal when external documentation link is clicked
    external_link_requested = Signal(str)  # url

    def __init__(self, parent=None):
        super().__init__(parent)

        self.docs_root = Path(__file__).parent.parent.parent / "docs"
        self.documentation_index: List[DocumentationEntry] = []
        self.ros2_doc_search = DocumentationSearch()  # Phase 12 enhancement

        self._setup_ui()
        self._load_documentation_index()

    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Header
        header = self._create_header()
        layout.addWidget(header)

        # Tab widget for local docs and ROS2 docs (Phase 12 enhancement)
        self.tab_widget = QTabWidget()

        # Tab 1: Local documentation
        local_docs_tab = self._create_local_docs_tab()
        self.tab_widget.addTab(local_docs_tab, "Project Docs")

        # Tab 2: ROS2 API documentation (Phase 12)
        ros2_docs_tab = self._create_ros2_docs_tab()
        self.tab_widget.addTab(ros2_docs_tab, "ROS2 Docs")

        layout.addWidget(self.tab_widget)

    def _create_local_docs_tab(self) -> QWidget:
        """Create local documentation tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # Search bar
        search_bar = self._create_search_bar()
        layout.addWidget(search_bar)

        # Main content
        splitter = QSplitter(Qt.Horizontal)

        # Left: Table of contents / search results
        left_panel = self._create_toc_panel()
        splitter.addWidget(left_panel)

        # Right: Documentation viewer
        right_panel = self._create_viewer_panel()
        splitter.addWidget(right_panel)

        splitter.setSizes([300, 700])
        layout.addWidget(splitter)

        return widget

    def _create_header(self) -> QWidget:
        """Create header"""
        header = QWidget()
        header.setStyleSheet("""
            QWidget {
                background-color: #009688;
                border-radius: 8px;
                padding: 15px;
            }
        """)

        layout = QVBoxLayout(header)

        title = QLabel("📚 Documentation Browser")
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        subtitle = QLabel("Search and browse RoboShire and ROS2 documentation")
        subtitle.setStyleSheet("color: rgba(255, 255, 255, 0.9);")
        layout.addWidget(subtitle)

        return header

    def _create_search_bar(self) -> QWidget:
        """Create search bar"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 5, 0, 5)

        # Search input
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Search documentation (e.g., 'URDF', 'node graph', 'build')...")
        self.search_input.textChanged.connect(self._search_documentation)
        layout.addWidget(self.search_input)

        # Category filter
        self.category_combo = QComboBox()
        self.category_combo.addItems([
            "All Categories",
            "Getting Started",
            "User Guides",
            "ROS2 Concepts",
            "Troubleshooting",
            "API Reference"
        ])
        self.category_combo.currentTextChanged.connect(self._filter_by_category)
        layout.addWidget(self.category_combo)

        # Search button
        search_btn = QPushButton("🔍 Search")
        search_btn.clicked.connect(lambda: self._search_documentation(self.search_input.text()))
        layout.addWidget(search_btn)

        return widget

    def _create_toc_panel(self) -> QWidget:
        """Create table of contents panel"""
        panel = QGroupBox("Documentation")
        layout = QVBoxLayout(panel)

        # Result count
        self.result_count_label = QLabel(f"{len(self.documentation_index)} documents")
        layout.addWidget(self.result_count_label)

        # TOC list
        self.toc_list = QListWidget()
        self.toc_list.currentItemChanged.connect(self._on_doc_selected)
        layout.addWidget(self.toc_list)

        # Quick links section
        quick_label = QLabel("⚡ Quick Links")
        quick_font = quick_label.font()
        quick_font.setBold(True)
        quick_label.setFont(quick_font)
        layout.addWidget(quick_label)

        quick_links = [
            ("Quick Start", "QUICK_START.md"),
            ("URDF Tutorial", "guides/urdf_to_robot.md"),
            ("MuJoCo Guide", "guides/mujoco_integration.md"),
            ("SSH Setup", "guides/ssh_setup.md"),
            ("Testing Guide", "testing/TESTING_GUIDE.md"),
        ]

        for title, path in quick_links:
            btn = QPushButton(f"📄 {title}")
            btn.setStyleSheet("""
                QPushButton {
                    text-align: left;
                    padding: 5px;
                    border: none;
                    background-color: transparent;
                }
                QPushButton:hover {
                    background-color: #e3f2fd;
                }
            """)
            btn.clicked.connect(lambda checked, p=path: self._load_document(p))
            layout.addWidget(btn)

        return panel

    def _create_viewer_panel(self) -> QWidget:
        """Create documentation viewer panel"""
        panel = QGroupBox("Content")
        layout = QVBoxLayout(panel)

        # Navigation buttons
        nav_layout = QHBoxLayout()

        self.back_btn = QPushButton("← Back")
        self.back_btn.clicked.connect(self._navigate_back)
        self.back_btn.setEnabled(False)
        nav_layout.addWidget(self.back_btn)

        self.forward_btn = QPushButton("Forward →")
        self.forward_btn.clicked.connect(self._navigate_forward)
        self.forward_btn.setEnabled(False)
        nav_layout.addWidget(self.forward_btn)

        nav_layout.addStretch()

        self.external_btn = QPushButton("🌐 Open in Browser")
        self.external_btn.clicked.connect(self._open_external)
        self.external_btn.setEnabled(False)
        nav_layout.addWidget(self.external_btn)

        layout.addLayout(nav_layout)

        # Text browser
        self.viewer = QTextBrowser()
        self.viewer.setOpenExternalLinks(False)
        self.viewer.anchorClicked.connect(self._on_link_clicked)
        self.viewer.setMarkdown(self._get_welcome_content())
        layout.addWidget(self.viewer)

        # History for navigation
        self.history: List[str] = []
        self.history_index = -1

        return panel

    def _load_documentation_index(self):
        """Load documentation index from docs/ directory"""
        if not self.docs_root.exists():
            return

        # Scan docs directory for markdown files
        for md_file in self.docs_root.rglob("*.md"):
            # Skip hidden files
            if md_file.name.startswith('.'):
                continue

            # Determine category from path
            rel_path = md_file.relative_to(self.docs_root)
            category = str(rel_path.parent) if rel_path.parent != Path('.') else "general"

            # Extract title from filename or first heading
            title = md_file.stem.replace('_', ' ').title()

            try:
                content = md_file.read_text(encoding='utf-8')
                # Try to get first heading as title
                for line in content.split('\n'):
                    if line.startswith('# '):
                        title = line.lstrip('# ').strip()
                        break

                entry = DocumentationEntry(title, str(rel_path), category)
                entry.content = content

                # Extract keywords for search
                entry.keywords = self._extract_keywords(content)

                self.documentation_index.append(entry)

            except Exception:
                continue

        # Populate TOC
        self._populate_toc()

    def _extract_keywords(self, content: str) -> List[str]:
        """Extract keywords from markdown content"""
        # Remove markdown syntax
        text = re.sub(r'[#*`\[\]()]', '', content)

        # Split into words
        words = re.findall(r'\b\w{4,}\b', text.lower())

        # Get unique words
        return list(set(words))

    def _populate_toc(self):
        """Populate table of contents"""
        self.toc_list.clear()

        for entry in self.documentation_index:
            item = QListWidgetItem(f"📄 {entry.title}")
            item.setData(Qt.UserRole, entry)
            self.toc_list.addItem(item)

        self.result_count_label.setText(f"{len(self.documentation_index)} documents")

    def _search_documentation(self, query: str):
        """Search documentation"""
        if not query.strip():
            self._populate_toc()
            return

        query_lower = query.lower()
        results = []

        for entry in self.documentation_index:
            # Search in title
            if query_lower in entry.title.lower():
                results.append(entry)
                continue

            # Search in keywords
            if any(query_lower in keyword for keyword in entry.keywords):
                results.append(entry)
                continue

        # Update TOC with results
        self.toc_list.clear()
        for entry in results:
            item = QListWidgetItem(f"📄 {entry.title}")
            item.setData(Qt.UserRole, entry)
            self.toc_list.addItem(item)

        self.result_count_label.setText(f"{len(results)} result(s) for '{query}'")

    def _filter_by_category(self, category: str):
        """Filter documentation by category"""
        if category == "All Categories":
            self._populate_toc()
            return

        # Map category names to folder names
        category_map = {
            "Getting Started": ".",
            "User Guides": "guides",
            "ROS2 Concepts": "concepts",
            "Troubleshooting": "troubleshooting",
            "API Reference": "api"
        }

        folder = category_map.get(category, "")

        self.toc_list.clear()
        for entry in self.documentation_index:
            if folder in entry.category or folder == "":
                item = QListWidgetItem(f"📄 {entry.title}")
                item.setData(Qt.UserRole, entry)
                self.toc_list.addItem(item)

    def _on_doc_selected(self, current: QListWidgetItem, previous: QListWidgetItem):
        """Handle documentation selection"""
        if not current:
            return

        entry = current.data(Qt.UserRole)
        if entry:
            self._load_document(entry.path)

    def _load_document(self, path: str):
        """Load and display document"""
        doc_path = self.docs_root / path

        if not doc_path.exists():
            self.viewer.setMarkdown(f"# Document Not Found\n\nPath: {path}")
            return

        try:
            content = doc_path.read_text(encoding='utf-8')
            self.viewer.setMarkdown(content)

            # Add to history
            self.history_index += 1
            self.history = self.history[:self.history_index]
            self.history.append(str(doc_path))

            self._update_navigation_buttons()

        except Exception as e:
            self.viewer.setMarkdown(f"# Error Loading Document\n\n{e}")

    def _navigate_back(self):
        """Navigate backward in history"""
        if self.history_index > 0:
            self.history_index -= 1
            doc_path = self.history[self.history_index]
            content = Path(doc_path).read_text(encoding='utf-8')
            self.viewer.setMarkdown(content)
            self._update_navigation_buttons()

    def _navigate_forward(self):
        """Navigate forward in history"""
        if self.history_index < len(self.history) - 1:
            self.history_index += 1
            doc_path = self.history[self.history_index]
            content = Path(doc_path).read_text(encoding='utf-8')
            self.viewer.setMarkdown(content)
            self._update_navigation_buttons()

    def _update_navigation_buttons(self):
        """Update navigation button states"""
        self.back_btn.setEnabled(self.history_index > 0)
        self.forward_btn.setEnabled(self.history_index < len(self.history) - 1)
        self.external_btn.setEnabled(len(self.history) > 0)

    def _open_external(self):
        """Open current document in external browser"""
        if self.history_index >= 0 and self.history_index < len(self.history):
            doc_path = self.history[self.history_index]
            self.external_link_requested.emit(f"file://{doc_path}")

    def _on_link_clicked(self, url: QUrl):
        """Handle link clicks"""
        url_str = url.toString()

        # Handle internal links (relative paths)
        if not url_str.startswith('http'):
            self._load_document(url_str)
        else:
            # External link
            self.external_link_requested.emit(url_str)

    def _get_welcome_content(self) -> str:
        """Get welcome content"""
        return """
# 📚 RoboShire Documentation

Welcome to the integrated documentation browser!

## Quick Access

- **[Quick Start Guide](QUICK_START.md)** - Get started in 10 minutes
- **[URDF to Robot Tutorial](guides/urdf_to_robot.md)** - Complete workflow guide
- **[MuJoCo Integration](guides/mujoco_integration.md)** - 3D visualization
- **[SSH Setup Guide](guides/ssh_setup.md)** - Configure your VM
- **[Testing Guide](testing/TESTING_GUIDE.md)** - Validate your robot

## How to Use

1. **Search**: Type keywords in the search box (e.g., "node graph", "build", "URDF")
2. **Browse**: Click on documents in the left panel
3. **Navigate**: Use Back/Forward buttons
4. **Filter**: Select a category to narrow results

## External Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [RoboShire GitHub](https://github.com/your-org/robotstudio)

---

💡 **Tip**: Press `F1` or `Ctrl+?` to open this browser anytime!
"""

    def search_and_show(self, query: str):
        """Search and show first result (for context-sensitive help)"""
        self._search_documentation(query)

        # Show first result if any
        if self.toc_list.count() > 0:
            self.toc_list.setCurrentRow(0)

    # Phase 12: ROS2 Documentation Tab
    def _create_ros2_docs_tab(self) -> QWidget:
        """Create ROS2 documentation tab (Phase 12)"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # Search bar
        search_layout = QHBoxLayout()
        search_layout.addWidget(QLabel("Search:"))

        self.ros2_search_input = QLineEdit()
        self.ros2_search_input.setPlaceholderText("Search ROS2 docs (e.g., 'topic', 'publisher', 'launch')...")
        self.ros2_search_input.returnPressed.connect(self._on_ros2_search)
        search_layout.addWidget(self.ros2_search_input)

        search_btn = QPushButton("Search")
        search_btn.clicked.connect(self._on_ros2_search)
        search_btn.setMaximumWidth(80)
        search_layout.addWidget(search_btn)

        layout.addLayout(search_layout)

        # Category filter
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("Category:"))

        self.ros2_category_combo = QComboBox()
        self.ros2_category_combo.addItem("All Categories", None)
        for category in DocCategory:
            self.ros2_category_combo.addItem(category.value.title(), category)
        self.ros2_category_combo.currentIndexChanged.connect(self._on_ros2_search)
        filter_layout.addWidget(self.ros2_category_combo)

        filter_layout.addStretch()

        self.ros2_stats_label = QLabel()
        self.ros2_stats_label.setStyleSheet("color: #666;")
        filter_layout.addWidget(self.ros2_stats_label)

        layout.addLayout(filter_layout)

        # Splitter for list and detail
        splitter = QSplitter(Qt.Horizontal)

        # Left: Results list
        list_container = QWidget()
        list_layout = QVBoxLayout(list_container)
        list_layout.setContentsMargins(0, 0, 0, 0)

        list_label = QLabel("Documentation Entries")
        list_font = QFont()
        list_font.setBold(True)
        list_label.setFont(list_font)
        list_layout.addWidget(list_label)

        self.ros2_results_list = QListWidget()
        self.ros2_results_list.setAlternatingRowColors(True)
        self.ros2_results_list.itemClicked.connect(self._on_ros2_entry_clicked)
        list_layout.addWidget(self.ros2_results_list)

        splitter.addWidget(list_container)

        # Right: Detail view
        detail_container = QWidget()
        detail_layout = QVBoxLayout(detail_container)
        detail_layout.setContentsMargins(0, 0, 0, 0)

        detail_label = QLabel("Details")
        detail_label.setFont(list_font)
        detail_layout.addWidget(detail_label)

        self.ros2_detail_browser = QTextBrowser()
        self.ros2_detail_browser.setOpenExternalLinks(False)
        self.ros2_detail_browser.anchorClicked.connect(self._on_ros2_link_clicked)
        detail_layout.addWidget(self.ros2_detail_browser)

        splitter.addWidget(detail_container)

        splitter.setSizes([400, 600])
        layout.addWidget(splitter)

        # Status
        self.ros2_status_label = QLabel("Ready - Browse 25+ ROS2 documentation entries")
        self.ros2_status_label.setStyleSheet("color: #666; font-size: 9pt;")
        layout.addWidget(self.ros2_status_label)

        # Load all docs initially
        self._load_all_ros2_docs()

        return widget

    def _load_all_ros2_docs(self):
        """Load all ROS2 documentation entries"""
        self.ros2_current_results = self.ros2_doc_search.entries
        self._display_ros2_results()
        self._update_ros2_stats()

    def _on_ros2_search(self):
        """Handle ROS2 documentation search"""
        query = self.ros2_search_input.text().strip()
        category_data = self.ros2_category_combo.currentData()

        self.ros2_current_results = self.ros2_doc_search.search(query, category_data)
        self._display_ros2_results()
        self._update_ros2_stats()

        if query:
            self.ros2_status_label.setText(f"Search: '{query}'")
        else:
            self.ros2_status_label.setText("Showing all entries")

    def _display_ros2_results(self):
        """Display ROS2 search results"""
        self.ros2_results_list.clear()

        for entry in self.ros2_current_results:
            # Create compact item text
            category_badge = f"[{entry.category.value[:4].upper()}]"
            item_text = f"{category_badge} {entry.title}"

            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, entry)
            self.ros2_results_list.addItem(item)

        if not self.ros2_current_results:
            item = QListWidgetItem("No documentation found. Try a different search term.")
            self.ros2_results_list.addItem(item)

    def _update_ros2_stats(self):
        """Update ROS2 statistics label"""
        total = self.ros2_doc_search.get_entry_count()
        showing = len(self.ros2_current_results)

        if showing == total:
            self.ros2_stats_label.setText(f"{total} entries")
        else:
            self.ros2_stats_label.setText(f"{showing} of {total} entries")

    def _on_ros2_entry_clicked(self, item: QListWidgetItem):
        """Handle ROS2 entry click"""
        entry = item.data(Qt.UserRole)
        if entry and isinstance(entry, DocEntry):
            self._show_ros2_entry_details(entry)

    def _show_ros2_entry_details(self, entry: DocEntry):
        """Show detailed information about ROS2 entry"""
        html = f"""
        <html>
        <body style="font-family: sans-serif; font-size: 11pt;">
            <h2 style="color: #2c3e50; margin-top: 0;">{entry.title}</h2>

            <p style="color: #7f8c8d; font-size: 10pt;">
                <b>Category:</b> {entry.category.value.title()}
            </p>

            <h3 style="color: #34495e;">Description</h3>
            <p style="line-height: 1.6;">{entry.description}</p>
        """

        if entry.package:
            html += f"""
            <h3 style="color: #34495e;">Package</h3>
            <p><code style="background: #ecf0f1; padding: 2px 6px; border-radius: 3px;">{entry.package}</code></p>
            """

        if entry.tags:
            tags_html = " ".join(f"<span style='background: #3498db; color: white; padding: 2px 8px; border-radius: 10px; font-size: 9pt; margin-right: 4px;'>#{tag}</span>" for tag in entry.tags)
            html += f"""
            <h3 style="color: #34495e;">Tags</h3>
            <p>{tags_html}</p>
            """

        if entry.keywords:
            keywords_html = ", ".join(f"<code>{kw}</code>" for kw in entry.keywords[:10])
            html += f"""
            <h3 style="color: #34495e;">Keywords</h3>
            <p style="font-size: 10pt;">{keywords_html}</p>
            """

        if entry.code_example:
            html += f"""
            <h3 style="color: #34495e;">Code Example</h3>
            <pre style="background: #2c3e50; color: #ecf0f1; padding: 12px; border-radius: 5px; overflow-x: auto; font-size: 9pt; white-space: pre-wrap;">{entry.code_example}</pre>
            """

        if entry.related:
            related_html = ", ".join(f"<a href='related:{rel}'>{rel}</a>" for rel in entry.related)
            html += f"""
            <h3 style="color: #34495e;">Related</h3>
            <p>{related_html}</p>
            """

        html += f"""
            <h3 style="color: #34495e;">Documentation</h3>
            <p><a href="{entry.url}" style="color: #3498db;">{entry.url}</a></p>

            <p style="margin-top: 20px;">
                <a href="open:{entry.url}" style="background: #3498db; color: white; padding: 8px 16px; text-decoration: none; border-radius: 4px; display: inline-block;">
                    Open in Browser
                </a>
            </p>
        </body>
        </html>
        """

        self.ros2_detail_browser.setHtml(html)

    def _on_ros2_link_clicked(self, url: QUrl):
        """Handle ROS2 link clicks"""
        url_str = url.toString()

        if url_str.startswith("open:"):
            # Open in browser
            actual_url = url_str[5:]
            self._open_ros2_url(actual_url)
        elif url_str.startswith("related:"):
            # Search for related entry
            related_title = url_str[8:]
            self.ros2_search_input.setText(related_title)
            self._on_ros2_search()
        else:
            # Regular URL
            self._open_ros2_url(url_str)

    def _open_ros2_url(self, url: str):
        """Open ROS2 URL in default browser"""
        try:
            webbrowser.open(url)
            self.ros2_status_label.setText(f"Opened: {url}")
        except Exception as e:
            QMessageBox.warning(
                self,
                "Error Opening URL",
                f"Failed to open URL:\n{url}\n\nError: {str(e)}"
            )
