import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="PyDynamixel",
    version="1.0",
    author="guichristmann",
    author_email="guichristmann@gmail.com",
    description="A package for managing motors that use the Dynamixel protocol",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/guichristmann/PyDynamixel_v2",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Linux",
    ],
    python_requires='>=3.5',
)
