import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pure_pursuit_baseline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',      
    ],
    zip_safe=True,
    maintainer='rudolf',
    maintainer_email='krecht.rudolf@ga.sze.hu',
    description='Pure baseline constant-speed pure pursuit controller',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit_baseline.pure_pursuit_node:main',
        ],
    },
)
