"""Packaging must identify one app/version and exclude backup/source artifacts."""
from pathlib import Path
import warnings

from imu_desktop import __version__
from setuptools.config.pyprojecttoml import read_configuration


def test_project_metadata_uses_runtime_version_and_only_app_packages():
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        config = read_configuration(str(Path(__file__).resolve().parents[1]/'pyproject.toml'))
    assert config['project']['version'] == __version__
    packages = config['tool']['setuptools']['packages']
    assert 'imu_desktop' in packages and 'imu_desktop.core' in packages
    assert all(name == 'imu_desktop' or name.startswith('imu_desktop.') for name in packages)
