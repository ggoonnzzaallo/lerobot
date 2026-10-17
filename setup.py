# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#
# Legacy flat layout: Python sources live at repo root (__init__.py, common/, configs/, scripts/)
# but imports use the lerobot.* namespace. setuptools package-dir maps those dirs accordingly.

from __future__ import annotations

from setuptools import find_namespace_packages, setup


def lerobot_packages() -> list[str]:
    pkgs = ["lerobot", "lerobot.common", "lerobot.configs", "lerobot.scripts"]
    for root, prefix in (
        ("common", "lerobot.common"),
        ("configs", "lerobot.configs"),
        ("scripts", "lerobot.scripts"),
    ):
        for p in find_namespace_packages(where=root):
            pkgs.append(f"{prefix}.{p}")
    return pkgs


PACKAGE_DIR = {
    "lerobot": ".",
    "lerobot.common": "common",
    "lerobot.configs": "configs",
    "lerobot.scripts": "scripts",
}

setup(packages=lerobot_packages(), package_dir=PACKAGE_DIR)
