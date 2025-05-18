from setuptools import setup, find_packages

# Read the requirements from the requirements.txt file
with open('requirements.txt', 'r') as f:
    requirements = f.read().splitlines()

setup(
    name="whole_body_mppi",  #
    version="1.0",  # Update version as needed
    description="A Python package for MPPI-based controllers for quadrupeds", 
    author="Juan Alvarez-Padilla",
    author_email="jrap.udg@gmail.com", 
    url="https://whole-body-mppi.github.io/", 
    packages=find_packages(),  
    install_requires=requirements, 
    python_requires=">=3.8", 
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
