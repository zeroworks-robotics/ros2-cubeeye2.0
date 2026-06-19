
from setuptools import setup, find_packages

setup(
    name="CubeEye",
    version="2.5.11",
    packages=find_packages(where="."),    
    package_dir={"":"."},
    package_data={"CubeEye": ["*.py", "*.so", "*.dll", "*.pyd"]},
    install_requires=[        
        "numpy>=1.23.5",
    ],
    python_requires=">=3.8",
)
