from pathlib import Path


def get_project_root():
    return Path(__file__).parent.parent


def get_default_assets_root():
    return get_project_root() / "assets"


def get_default_config_root():
    return get_project_root() / "configs"


DEFAULT_CONFIG_DIR = get_default_config_root()
DEFAULT_ASSET_DIR = get_default_assets_root()
