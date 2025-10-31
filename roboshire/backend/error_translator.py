"""
Error Message Translator - Translate compiler/build errors to human-readable messages

Parses common ROS2 build errors and provides beginner-friendly explanations.

Author: RoboShire Team
Version: 0.14.0
"""

import re
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class ErrorPattern:
    """Pattern for matching and translating errors"""
    pattern: str  # Regex pattern
    category: str  # Error category
    title: str  # User-friendly title
    explanation: str  # What the error means
    solution: str  # How to fix it
    priority: int = 1  # Higher = check first


class ErrorTranslator:
    """
    Translates technical error messages into beginner-friendly explanations

    Features:
    - Pattern matching for common errors
    - Context-aware suggestions
    - Links to documentation
    - Code fix suggestions
    """

    def __init__(self):
        self.patterns: List[ErrorPattern] = []
        self._load_error_patterns()

    def _load_error_patterns(self):
        """Load all known error patterns"""

        # Python errors
        self.patterns.extend([
            ErrorPattern(
                pattern=r"ModuleNotFoundError: No module named '([^']+)'",
                category="Python Import",
                title="Missing Python Module",
                explanation="Python can't find the module '{match_1}'. This usually means the package isn't installed.",
                solution="""
**Solution:**
1. Check if the package is in your `package.xml`:
   ```xml
   <depend>rclpy</depend>
   ```

2. Install missing Python packages:
   ```bash
   pip install {match_1}
   ```

3. For ROS2 packages, ensure you've sourced the workspace:
   ```bash
   source install/setup.bash
   ```
                """,
                priority=5
            ),

            ErrorPattern(
                pattern=r"ImportError: cannot import name '([^']+)' from '([^']+)'",
                category="Python Import",
                title="Cannot Import Name",
                explanation="Python found the module '{match_2}' but couldn't find '{match_1}' inside it.",
                solution="""
**Solution:**
1. Check if '{match_1}' exists in '{match_2}'
2. Verify the spelling (case-sensitive!)
3. Check if you're using the correct version of the package
4. Try updating the package: `pip install --upgrade {match_2}`
                """,
                priority=4
            ),

            ErrorPattern(
                pattern=r"IndentationError: (.*)",
                category="Python Syntax",
                title="Indentation Error",
                explanation="Python detected incorrect indentation. Python requires consistent spacing (usually 4 spaces).",
                solution="""
**Solution:**
1. Check the file mentioned in the error for mixed tabs/spaces
2. Use your editor's "Show Whitespace" feature
3. In VSCode: Select all (Ctrl+A) → Format Document (Shift+Alt+F)
4. Python requires 4 spaces for each indentation level
                """,
                priority=5
            ),

            ErrorPattern(
                pattern=r"SyntaxError: (.*)",
                category="Python Syntax",
                title="Python Syntax Error",
                explanation="Python found invalid syntax: {match_1}",
                solution="""
**Solution:**
1. Check for missing colons (:) at end of if/for/def statements
2. Verify all parentheses, brackets, and quotes are closed
3. Check for reserved keywords used as variable names
4. Review the file at the line number mentioned in the error
                """,
                priority=5
            ),
        ])

        # C++ compilation errors
        self.patterns.extend([
            ErrorPattern(
                pattern=r"fatal error: ([^\s]+): No such file or directory",
                category="C++ Compilation",
                title="Missing Header File",
                explanation="The compiler can't find the header file '{match_1}'.",
                solution="""
**Solution:**
1. Check if the package providing '{match_1}' is in your `package.xml`:
   ```xml
   <depend>rclcpp</depend>
   ```

2. Verify the include path in your CMakeLists.txt:
   ```cmake
   find_package(rclcpp REQUIRED)
   ament_target_dependencies(your_node rclcpp)
   ```

3. For system libraries, install the development package:
   ```bash
   sudo apt install lib[name]-dev
   ```
                """,
                priority=5
            ),

            ErrorPattern(
                pattern=r"undefined reference to `([^']+)'",
                category="C++ Linking",
                title="Undefined Reference (Linker Error)",
                explanation="The linker can't find the implementation of '{match_1}'.",
                solution="""
**Solution:**
1. Add missing library to CMakeLists.txt:
   ```cmake
   target_link_libraries(your_node your_library)
   ```

2. Check if you declared but didn't implement a function
3. Verify all required packages are in find_package():
   ```cmake
   find_package(rclcpp REQUIRED)
   ```

4. Ensure the library is installed on your system
                """,
                priority=4
            ),

            ErrorPattern(
                pattern=r"error: '([^']+)' was not declared in this scope",
                category="C++ Compilation",
                title="Undeclared Identifier",
                explanation="The compiler doesn't know about '{match_1}'. It might be misspelled or missing an include.",
                solution="""
**Solution:**
1. Check spelling of '{match_1}' (C++ is case-sensitive!)

2. Add missing include:
   ```cpp
   #include <rclcpp/rclcpp.hpp>
   ```

3. Add namespace:
   ```cpp
   using namespace std;
   // or use std::{match_1}
   ```

4. Ensure the header is included before use
                """,
                priority=5
            ),

            ErrorPattern(
                pattern=r"error: no matching function for call to '([^']+)'",
                category="C++ Compilation",
                title="Function Call Mismatch",
                explanation="You're calling '{match_1}' with wrong arguments or the function doesn't exist.",
                solution="""
**Solution:**
1. Check the function signature (number and types of arguments)
2. Verify you're passing the correct types
3. Check documentation for the correct API usage
4. Ensure the function is defined before calling it
                """,
                priority=4
            ),
        ])

        # CMake errors
        self.patterns.extend([
            ErrorPattern(
                pattern=r"Could not find a package configuration file provided by \"([^\"]+)\"",
                category="CMake",
                title="Package Not Found",
                explanation="CMake can't find the ROS2 package '{match_1}'.",
                solution="""
**Solution:**
1. Install the missing package:
   ```bash
   sudo apt install ros-humble-{match_1}
   ```

2. Add to package.xml:
   ```xml
   <depend>{match_1}</depend>
   ```

3. Source your workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

4. If it's a custom package, build it first:
   ```bash
   colcon build --packages-select {match_1}
   ```
                """,
                priority=5
            ),

            ErrorPattern(
                pattern=r"CMake Error: The following variables are used in this project, but they are set to NOTFOUND:\n.*?\n.*?([^\n]+)",
                category="CMake",
                title="CMake Variable Not Found",
                explanation="CMake variable is set to NOTFOUND. A required file or library is missing.",
                solution="""
**Solution:**
1. Check if all required packages are installed
2. Verify paths in CMakeLists.txt are correct
3. Re-run CMake with clean build:
   ```bash
   rm -rf build install
   colcon build
   ```
4. Install missing development libraries
                """,
                priority=3
            ),
        ])

        # colcon/build system errors
        self.patterns.extend([
            ErrorPattern(
                pattern=r"Package '([^']+)' not found",
                category="ROS2 Package",
                title="ROS2 Package Not Found",
                explanation="The ROS2 package '{match_1}' is not installed or not in your workspace.",
                solution="""
**Solution:**
1. Install from apt:
   ```bash
   sudo apt install ros-humble-{match_1}
   ```

2. Or build it in your workspace:
   ```bash
   cd ~/workspace/src
   git clone [package-repo-url]
   cd ~/workspace
   colcon build
   ```

3. Check package name spelling (use underscores, not hyphens)
                """,
                priority=5
            ),

            ErrorPattern(
                pattern=r"SetuptoolsDeprecationWarning: (.*)",
                category="Python Build",
                title="Setuptools Warning",
                explanation="Setuptools version warning: {match_1}. Usually safe to ignore.",
                solution="""
**Solution (Optional):**
This is usually just a warning and won't stop your build.

To fix:
1. Update setuptools:
   ```bash
   pip install --upgrade setuptools
   ```

2. Or add to package.xml:
   ```xml
   <depend>python3-setuptools</depend>
   ```
                """,
                priority=1
            ),
        ])

        # XML/package.xml errors
        self.patterns.extend([
            ErrorPattern(
                pattern=r"Error\(s\) in package '([^']+)'.*?InvalidPackage: (.*)",
                category="package.xml",
                title="Invalid package.xml",
                explanation="The package.xml file for '{match_1}' has errors: {match_2}",
                solution="""
**Solution:**
1. Open the package.xml file and check syntax
2. Ensure all tags are properly closed
3. Verify required fields: name, version, description, maintainer, license

Example valid package.xml:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Description</description>
  <maintainer email="user@example.com">Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
</package>
```
                """,
                priority=5
            ),
        ])

        # Permission/access errors
        self.patterns.extend([
            ErrorPattern(
                pattern=r"Permission denied|permission denied",
                category="System",
                title="Permission Denied",
                explanation="You don't have permission to access a file or directory.",
                solution="""
**Solution:**
1. Check file permissions:
   ```bash
   ls -la [filename]
   ```

2. For workspace files, ensure correct ownership:
   ```bash
   sudo chown -R $USER:$USER ~/workspace
   ```

3. For system operations, use sudo:
   ```bash
   sudo apt install [package]
   ```

4. Never run colcon build with sudo!
                """,
                priority=4
            ),
        ])

        # Sort patterns by priority (highest first)
        self.patterns.sort(key=lambda p: p.priority, reverse=True)

    def translate(self, error_text: str) -> List[Dict[str, str]]:
        """
        Translate error text into user-friendly explanations

        Args:
            error_text: Raw error text from compiler/build system

        Returns:
            List of error translations with suggestions
        """
        translations = []

        for pattern in self.patterns:
            matches = list(re.finditer(pattern.pattern, error_text, re.MULTILINE | re.IGNORECASE))

            for match in matches:
                # Extract matched groups
                groups = match.groups()

                # Replace placeholders in explanation and solution
                explanation = pattern.explanation
                solution = pattern.solution

                for i, group in enumerate(groups, 1):
                    explanation = explanation.replace(f"{{match_{i}}}", group)
                    solution = solution.replace(f"{{match_{i}}}", group)

                translation = {
                    'category': pattern.category,
                    'title': pattern.title,
                    'explanation': explanation,
                    'solution': solution,
                    'original_error': match.group(0),
                    'line_number': self._find_line_number(error_text, match.start())
                }

                translations.append(translation)

        # If no patterns matched, provide generic help
        if not translations:
            translations.append(self._generic_error_help(error_text))

        return translations

    def _find_line_number(self, text: str, position: int) -> int:
        """Find line number for a position in text"""
        return text[:position].count('\n') + 1

    def _generic_error_help(self, error_text: str) -> Dict[str, str]:
        """Provide generic help when no pattern matches"""

        # Try to detect error type
        if 'error:' in error_text.lower():
            category = "Build Error"
        elif 'warning:' in error_text.lower():
            category = "Build Warning"
        else:
            category = "Build Output"

        return {
            'category': category,
            'title': "Build Error (No specific pattern matched)",
            'explanation': "An error occurred during the build, but it doesn't match any known patterns.",
            'solution': """
**General Troubleshooting Steps:**

1. **Read the full error message** - The details matter!

2. **Check recent changes** - What did you modify?

3. **Clean and rebuild**:
   ```bash
   rm -rf build install
   colcon build
   ```

4. **Check syntax** - Verify Python/C++ syntax is correct

5. **Verify dependencies** - Ensure all required packages are installed

6. **Search online** - Copy the error message and search:
   - ROS Answers: https://answers.ros.org
   - Stack Overflow
   - GitHub Issues

7. **Ask for help** - Provide:
   - Full error message
   - What you were trying to do
   - Your code (relevant parts)
   - ROS2 version (Humble, etc.)
            """,
            'original_error': error_text[:500],  # First 500 chars
            'line_number': 0
        }

    def get_category_icon(self, category: str) -> str:
        """Get an icon for error category"""
        icons = {
            'Python Import': '🐍',
            'Python Syntax': '🐍',
            'Python Build': '🐍',
            'C++ Compilation': '⚙️',
            'C++ Linking': '🔗',
            'CMake': '🏗️',
            'ROS2 Package': '📦',
            'package.xml': '📄',
            'System': '💻',
            'Build Error': '❌',
            'Build Warning': '⚠️'
        }
        return icons.get(category, '❓')


# Standalone test
if __name__ == "__main__":
    translator = ErrorTranslator()

    # Test cases
    test_errors = [
        "ModuleNotFoundError: No module named 'rclpy'",
        "fatal error: rclcpp/rclcpp.hpp: No such file or directory",
        "error: 'Node' was not declared in this scope",
        "Could not find a package configuration file provided by \"geometry_msgs\"",
        "IndentationError: expected an indented block",
    ]

    for error in test_errors:
        print(f"\n{'='*60}")
        print(f"Original Error: {error}")
        print('='*60)

        translations = translator.translate(error)
        for translation in translations:
            print(f"\n{translator.get_category_icon(translation['category'])} {translation['title']}")
            print(f"Category: {translation['category']}")
            print(f"\nExplanation:\n{translation['explanation']}")
            print(f"\nSolution:\n{translation['solution']}")
