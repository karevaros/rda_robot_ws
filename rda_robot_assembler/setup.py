from setuptools import setup

package_name = "rda_robot_assembler"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kim",
    maintainer_email="akswnddl255@gmail.com",
    description="RDA 로봇 어셈블러 — 파트 결합 인터랙티브 조립 GUI (mounts.yaml 생성)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "assembler = rda_robot_assembler.app:main",
        ],
    },
)
