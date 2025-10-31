"""
RoboShire Resources Module

Provides access to branding assets, icons, and other resources.
"""

from pathlib import Path

# Resource directories
RESOURCES_DIR = Path(__file__).parent
BRANDING_DIR = RESOURCES_DIR / "branding"
EXAMPLES_DIR = RESOURCES_DIR / "examples"

# Logo files
LOGO_WITH_TEXT = BRANDING_DIR / "logowithtext.jpeg"
LOGO_WITHOUT_TEXT = BRANDING_DIR / "logowithouttext.jpeg"

# Brand colors (from brand identity)
BRAND_COLORS = {
    "olive_green": "#556B2F",  # Primary brand color
    "olive_green_rgb": (85, 107, 47),
    "white": "#FFFFFF",
    "beige_background": "#F5F5DC",  # Cream background from logo
}

# Typography (from brand identity)
BRAND_FONTS = {
    "primary": "Montserrat SemiBold",  # Headings, titles, labels
    "secondary": "Montserrat Regular",  # Body text, menus
    "monospace": "JetBrains Mono",  # Code elements
}

def get_logo_path(with_text: bool = True) -> Path:
    """
    Get path to logo file

    Args:
        with_text: If True, returns logo with "ROBOSHIRE" text
                   If False, returns symbol-only logo

    Returns:
        Path to logo file
    """
    return LOGO_WITH_TEXT if with_text else LOGO_WITHOUT_TEXT
