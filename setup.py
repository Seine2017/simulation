from setuptools import setup

setup(
  name = "simulation",
  version = "0.1",
  platforms = "ALL",

  packages = ["simulation"],
  entry_points = {
    "console_scripts": [
      "simulation = simulation:main",
    ],
  },

  install_requires = [
    "pyode==1.2.0",
  ],
)
