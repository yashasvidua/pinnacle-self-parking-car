"""Loads config.yaml (+ .env overrides) and writes sections back without losing comments."""
import copy
import os
from pathlib import Path

from dotenv import load_dotenv
from ruamel.yaml import YAML

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = REPO_ROOT / "config.yaml"

_yaml = YAML()
_yaml.preserve_quotes = True
_yaml.default_flow_style = None


def _to_plain(obj):
    """ruamel CommentedMap/Seq -> plain dict/list."""
    if isinstance(obj, dict):
        return {str(k): _to_plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_plain(v) for v in obj]
    return obj


def load_config(path=None):
    """Return the config as a plain dict, with CAM_URL / DEVKIT_IP from .env applied."""
    path = Path(path) if path else CONFIG_PATH
    with open(path, "r", encoding="utf-8") as f:
        cfg = _to_plain(_yaml.load(f)) or {}
    load_dotenv(REPO_ROOT / ".env")
    net = cfg.setdefault("network", {})
    if os.getenv("CAM_URL"):
        net["cam_url"] = os.getenv("CAM_URL")
    if os.getenv("DEVKIT_IP"):
        net["devkit_ip"] = os.getenv("DEVKIT_IP")
    return cfg


def update_config_section(section, values, path=None):
    """Merge `values` into the top-level `section` of config.yaml, keeping comments."""
    path = Path(path) if path else CONFIG_PATH
    with open(path, "r", encoding="utf-8") as f:
        doc = _yaml.load(f)
    if doc.get(section) is None:
        doc[section] = {}
    for k, v in values.items():
        doc[section][k] = copy.deepcopy(v)
    with open(path, "w", encoding="utf-8") as f:
        _yaml.dump(doc, f)
