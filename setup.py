from setuptools import find_packages
from setuptools import setup

package_name = 'ibvs_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=['ros2connect', 'controller'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'matplotlib', 'numpy'],
    zip_safe=True,
    author='Alex Utkin',
    author_email='xai@yandex.ru',
    maintainer='Alex Utkin',
    maintainer_email='xai@yandex.ru',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ros2 controller with IBVS methods',
    license='Apache License, Version 2.0',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'control = controller.controller:main',
            'capture = controller.controller:capture',
            'move_predator = controller.controller:move_predator',
            'chase_key = controller.controller:chase_key'
        ],
    },
)
