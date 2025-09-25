#!/usr/bin/env python3
"""
Setup script for Crazyflie Autonomous Flight Package
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read the README file
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

# Read requirements
requirements = []
with open('requirements.txt') as f:
    requirements = [line.strip() for line in f if line.strip() and not line.startswith('#')]

setup(
    name="cfpilot",
    version="1.0.0",
    author="Crazyflie Autonomous Team",
    author_email="",
    description="Advanced autonomous flight system for Crazyflie platform",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/your-repo/cfpilot",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: System :: Hardware :: Hardware Drivers",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=7.0",
            "pytest-cov>=4.0",
            "black>=22.0",
            "flake8>=5.0",
            "mypy>=1.0",
        ],
        "visualization": [
            "matplotlib>=3.5.0",
            "plotly>=5.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "cfpilot=cfpilot.cli:main",
        ],
    },
    include_package_data=True,
    package_data={
        "cfpilot": ["config/*.yaml"],
    },
)
