from glob import glob
import os

from setuptools import find_packages, setup


package_name = "sobits_open_mine"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "visual_prompts"),
            glob("visual_prompts/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="haruto",
    maintainer_email="e2458119@soka-u.jp",
    description=(
        "Bring Me task package with YOLOE perception "
        "and hsrb_library grasp."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "bring_me_node = sobits_open_mine.bring_me_main:main",
            "bring_me = sobits_open_mine.bring_me_main:main",
            (
                "yoloe_visual_prompt_node = "
                "sobits_open_mine.yoloe_visual_prompt_node:main"
            ),
            (
                "visual_prompt_selector = "
                "sobits_open_mine.visual_prompt_selector:main"
            ),
            (
                "grasp_test_node = "
                "sobits_open_mine.grasp_test_node:main"
            ),
        ],
    },
)
