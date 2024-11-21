from setuptools import setup, find_packages

# Read the requirements from the requirements.txt file
with open('requirements.txt', 'r') as f:
    requirements = f.read().splitlines()

setup(
    name="real_time_whole_body_mppi",  # Replace with your project name
    version="0.1.0",  # Update version as needed
    description="A Python package for MPPI-based controllers for quadrupeds",  # Update description
    author="Juan Alvarez-Padilla",  # Replace with your name
    author_email="jralvare@andrew.cmu.edu",  # Replace with your email
    url="https://whole-body-mppi.github.io/", 
    packages=find_packages(),  
    install_requires=requirements, 
    python_requires=">=3.9", 
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    entry_points={
        'console_scripts': [
        ],
    },
)
