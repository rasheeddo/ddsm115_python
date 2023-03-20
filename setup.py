from setuptools import setup

setup(
    name = "ddsm115",
    version = "0.0.1",
    author = "Rasheed Kittinanthapanya",
    author_email = "rasheedo.kit@gmail.com",
    url='https://github.com/rasheeddo/ddsm115_python', 
    license='MIT',
    packages=['ddsm115',],
    install_requires = [
        'crcmod',
        'numpy'
    ]
)