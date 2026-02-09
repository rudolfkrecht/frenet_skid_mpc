from setuptools import setup

package_name = 'frenet_skid_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/frenet_skid_mpc.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rudolf Krecht',
    maintainer_email='krecht.rudolf@ga.sze.hu',
    description='Frenet-based MPC controller for skid-steer robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frenet_skid_mpc_node = frenet_skid_mpc.frenet_skid_mpc_node:main',
        ],
    },
)
