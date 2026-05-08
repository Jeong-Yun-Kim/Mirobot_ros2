from pathlib import Path
from setuptools import find_packages, setup

package_name = "mirobot_order_delivery"
root = Path(__file__).parent


def data(pattern: str):
    return [str(p) for p in root.glob(pattern) if p.is_file()]


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "requirements.txt"]),
        (f"share/{package_name}/config", data("config/*")),
        (f"share/{package_name}/launch", data("launch/*")),
        (f"share/{package_name}/data", data("data/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Laptop-side Mirobot order worker with YOLO, pump control, dynamic calibration, and domain_bridge support.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "delivery_node = mirobot_order_delivery.delivery_node:main",
            "simulate_order = mirobot_order_delivery.simulate_order:main",
        ],
    },
)
