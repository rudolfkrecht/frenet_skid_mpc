from setuptools import setup

package_name = 'path_to_micro_waypoints'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/path_to_micro_waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rudolf Krecht',
    maintainer_email='krecht.rudolf@ga.sze.hu',
    description='Resample a Path into uniformly spaced micro-waypoints (PoseArray + Path + Marker).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_to_micro_waypoints_node = path_to_micro_waypoints.path_to_micro_waypoints_node:main',
        ],
    },
)
