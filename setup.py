from setuptools import setup, find_packages

setup(
    name="vr_robot_control",
    version="0.0.1",
    description="A package to control a robot using VR hand pose data from Quest",
    author="Parsa Bakhshandeh",
    author_email="parsa.bakhshandeh@sanctuary.ai",  # Replace with your email
    packages=find_packages(),
    install_requires=[
        "numpy",  # Required for matrix operations
    ],
    # entry_points={
    #     "console_scripts": ["vr_robot_control=vr_robot_control.vr_robot_control:main"]
    # },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
