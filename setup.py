import setuptools

setuptools.setup(
    name="mmint_utils",
    version="0.0.1",
    packages=["mmint_utils", "mmint_utils.gamma_helpers"],
    url="https://github.com/MMintLab/mmint_utils",
    description="MMint Lab Utility Functions/Helpers",
    install_requires=[
        'numpy',
        'pyyaml',
    ]
)
