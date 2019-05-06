from setuptools import setup

package_name = 'vector_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[],
    install_requires=['setuptools'],
    author='Chris Chan',
    author_email='xxx@xxx.org',
    maintainer='xxx xxx',
    maintainer_email='xxx@xxx.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package for Anki Vector.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'vision = scripts.vision:main',
            'movement = scripts.movement:main',
            'mopper = scripts.mopper:main'
        ],
    },
)