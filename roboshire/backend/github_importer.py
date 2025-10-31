"""
GitHub Package Importer

Import ROS2 packages from GitHub repositories:
- Clone repository
- Detect package structure
- Install dependencies
- Build package

Author: RoboShire Team
Phase: 11 (Advanced Features)
"""

from pathlib import Path
from typing import Optional, Tuple, List, Callable
import subprocess
import re
import xml.etree.ElementTree as ET


class GitHubImporter:
    """
    Import ROS2 packages from GitHub
    """

    def __init__(self, workspace_path: str):
        self.workspace_path = Path(workspace_path)
        self.src_path = self.workspace_path / "src"

    def validate_github_url(self, url: str) -> Tuple[bool, str]:
        """
        Validate GitHub URL format

        Returns:
            (is_valid, error_message)
        """
        # Accept formats:
        # - https://github.com/user/repo
        # - https://github.com/user/repo.git
        # - git@github.com:user/repo.git

        patterns = [
            r"^https://github\.com/[\w-]+/[\w-]+(\.git)?$",
            r"^git@github\.com:[\w-]+/[\w-]+(\.git)?$"
        ]

        for pattern in patterns:
            if re.match(pattern, url):
                return True, ""

        return False, "Invalid GitHub URL format. Expected: https://github.com/user/repo"

    def extract_repo_name(self, url: str) -> str:
        """Extract repository name from GitHub URL"""
        # Remove .git suffix if present
        url = url.rstrip('/')
        if url.endswith('.git'):
            url = url[:-4]

        # Extract last part of path
        return url.split('/')[-1]

    def clone_repository(
        self,
        github_url: str,
        branch: Optional[str] = None,
        progress_callback: Optional[Callable[[str], None]] = None
    ) -> Tuple[bool, str, Optional[Path]]:
        """
        Clone GitHub repository to workspace/src

        Args:
            github_url: GitHub repository URL
            branch: Specific branch to clone (default: main/master)
            progress_callback: Callback for progress updates

        Returns:
            (success, message, cloned_path)
        """
        # Validate URL
        is_valid, error = self.validate_github_url(github_url)
        if not is_valid:
            return False, error, None

        # Ensure src directory exists
        self.src_path.mkdir(parents=True, exist_ok=True)

        # Extract repo name
        repo_name = self.extract_repo_name(github_url)
        clone_path = self.src_path / repo_name

        # Check if already exists
        if clone_path.exists():
            return False, f"Directory already exists: {clone_path}", None

        # Build git clone command
        cmd = ["git", "clone", github_url]
        if branch:
            cmd.extend(["-b", branch])
        cmd.append(str(clone_path))

        if progress_callback:
            progress_callback(f"Cloning {github_url}...")

        try:
            # Run git clone
            result = subprocess.run(
                cmd,
                cwd=str(self.src_path),
                capture_output=True,
                text=True,
                timeout=300  # 5 minutes timeout
            )

            if result.returncode != 0:
                return False, f"Git clone failed: {result.stderr}", None

            if progress_callback:
                progress_callback(f"Cloned to {clone_path}")

            return True, f"Successfully cloned {repo_name}", clone_path

        except subprocess.TimeoutExpired:
            return False, "Clone timeout (5 minutes exceeded)", None
        except FileNotFoundError:
            return False, "Git not found. Please install Git.", None
        except Exception as e:
            return False, f"Clone failed: {e}", None

    def detect_ros_packages(self, repo_path: Path) -> List[Path]:
        """
        Detect ROS2 packages in cloned repository

        Looks for package.xml files

        Returns:
            List of paths to packages (directories containing package.xml)
        """
        packages = []

        # Search for package.xml files
        for package_xml in repo_path.rglob("package.xml"):
            package_dir = package_xml.parent
            packages.append(package_dir)

        return packages

    def parse_package_xml(self, package_path: Path) -> Optional[dict]:
        """
        Parse package.xml to extract metadata

        Returns:
            dict with package info or None if parsing fails
        """
        package_xml = package_path / "package.xml"

        if not package_xml.exists():
            return None

        try:
            tree = ET.parse(package_xml)
            root = tree.getroot()

            info = {
                "name": root.find("name").text if root.find("name") is not None else "unknown",
                "version": root.find("version").text if root.find("version") is not None else "0.0.0",
                "description": root.find("description").text if root.find("description") is not None else "",
                "maintainer": root.find("maintainer").text if root.find("maintainer") is not None else "unknown",
                "license": root.find("license").text if root.find("license") is not None else "unknown",
                "dependencies": []
            }

            # Extract dependencies
            for dep_type in ["depend", "build_depend", "exec_depend"]:
                for dep in root.findall(dep_type):
                    if dep.text and dep.text not in info["dependencies"]:
                        info["dependencies"].append(dep.text)

            return info

        except Exception as e:
            return None

    def install_dependencies(
        self,
        package_path: Path,
        ros_distro: str = "humble",
        progress_callback: Optional[Callable[[str], None]] = None
    ) -> Tuple[bool, str]:
        """
        Install package dependencies using rosdep

        Args:
            package_path: Path to package directory
            ros_distro: ROS2 distribution name
            progress_callback: Progress callback

        Returns:
            (success, message)
        """
        if progress_callback:
            progress_callback("Installing dependencies with rosdep...")

        try:
            # Run rosdep install
            cmd = [
                "rosdep", "install",
                "--from-paths", str(package_path),
                "--ignore-src",
                "-y",
                "--rosdistro", ros_distro
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=600  # 10 minutes timeout
            )

            if result.returncode != 0:
                # rosdep may return non-zero even on success (warnings)
                # Check if there are actual errors
                if "ERROR" in result.stderr:
                    return False, f"rosdep failed: {result.stderr}"

            if progress_callback:
                progress_callback("Dependencies installed")

            return True, "Dependencies installed successfully"

        except FileNotFoundError:
            return False, "rosdep not found. Install with: sudo apt install python3-rosdep"
        except subprocess.TimeoutExpired:
            return False, "Dependency installation timeout (10 minutes exceeded)"
        except Exception as e:
            return False, f"Dependency installation failed: {e}"

    def import_from_github(
        self,
        github_url: str,
        branch: Optional[str] = None,
        install_deps: bool = True,
        ros_distro: str = "humble",
        progress_callback: Optional[Callable[[str], None]] = None
    ) -> Tuple[bool, str, List[str]]:
        """
        Complete import workflow from GitHub

        Args:
            github_url: GitHub repository URL
            branch: Branch to clone
            install_deps: Whether to install dependencies
            ros_distro: ROS2 distribution
            progress_callback: Progress callback

        Returns:
            (success, message, list_of_package_names)
        """
        # Step 1: Clone repository
        success, message, clone_path = self.clone_repository(
            github_url, branch, progress_callback
        )

        if not success:
            return False, message, []

        # Step 2: Detect packages
        if progress_callback:
            progress_callback("Detecting ROS2 packages...")

        packages = self.detect_ros_packages(clone_path)

        if not packages:
            return False, f"No ROS2 packages found in {clone_path}", []

        if progress_callback:
            progress_callback(f"Found {len(packages)} package(s)")

        # Step 3: Parse package info
        package_names = []
        for pkg_path in packages:
            info = self.parse_package_xml(pkg_path)
            if info:
                package_names.append(info["name"])
                if progress_callback:
                    progress_callback(f"  - {info['name']}: {info['description']}")

        # Step 4: Install dependencies (optional)
        if install_deps:
            for pkg_path in packages:
                success, dep_message = self.install_dependencies(
                    pkg_path, ros_distro, progress_callback
                )
                if not success:
                    # Continue even if deps fail (user can install manually)
                    if progress_callback:
                        progress_callback(f"Warning: {dep_message}")

        # Success
        summary = (
            f"Successfully imported from GitHub!\n\n"
            f"Repository: {self.extract_repo_name(github_url)}\n"
            f"Location: {clone_path}\n"
            f"Packages: {', '.join(package_names)}\n\n"
            f"Next steps:\n"
            f"1. Build with colcon (Ctrl+B)\n"
            f"2. Test the package"
        )

        return True, summary, package_names

    def update_repository(
        self,
        repo_path: Path,
        progress_callback: Optional[Callable[[str], None]] = None
    ) -> Tuple[bool, str]:
        """
        Update (git pull) an existing repository

        Args:
            repo_path: Path to repository
            progress_callback: Progress callback

        Returns:
            (success, message)
        """
        if not repo_path.exists():
            return False, f"Repository not found: {repo_path}"

        if not (repo_path / ".git").exists():
            return False, f"Not a git repository: {repo_path}"

        if progress_callback:
            progress_callback(f"Updating {repo_path.name}...")

        try:
            # Run git pull
            result = subprocess.run(
                ["git", "pull"],
                cwd=str(repo_path),
                capture_output=True,
                text=True,
                timeout=300  # 5 minutes
            )

            if result.returncode != 0:
                return False, f"Git pull failed: {result.stderr}"

            if progress_callback:
                progress_callback("Update complete")

            return True, result.stdout or "Already up to date"

        except subprocess.TimeoutExpired:
            return False, "Update timeout (5 minutes exceeded)"
        except Exception as e:
            return False, f"Update failed: {e}"

    def list_imported_repos(self) -> List[dict]:
        """
        List all imported GitHub repositories

        Returns:
            List of dicts with repo info
        """
        repos = []

        if not self.src_path.exists():
            return repos

        for item in self.src_path.iterdir():
            if item.is_dir() and (item / ".git").exists():
                # Get git remote URL
                try:
                    result = subprocess.run(
                        ["git", "remote", "get-url", "origin"],
                        cwd=str(item),
                        capture_output=True,
                        text=True,
                        timeout=5
                    )

                    if result.returncode == 0:
                        url = result.stdout.strip()
                        repos.append({
                            "name": item.name,
                            "path": str(item),
                            "url": url
                        })
                except Exception:
                    pass

        return repos
