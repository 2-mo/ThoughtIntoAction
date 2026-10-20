"""Shared pytest options. Loaded for every suite under tests/, so it uses only pytest."""

import pytest


def pytest_addoption(parser):
    parser.addoption("--run-hardware", action="store_true", help="run tests marked hardware (real devices)")


def pytest_collection_modifyitems(config, items):
    if config.getoption("--run-hardware"):
        return
    skip = pytest.mark.skip(reason="needs real devices; pass --run-hardware")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)
