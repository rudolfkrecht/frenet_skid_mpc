from setuptools import setup

package_name = 'dynamic_uniform_spline_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/dynamic_uniform_spline_publisher.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rudolf Krecht',
    maintainer_email='krecht.rudolf@ga.sze.hu',
    description='Publishes curvature-feasible uniform spline path and v_ref',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dynamic_uniform_spline_publisher_node = dynamic_uniform_spline_publisher.dynamic_uniform_spline_publisher_node:main',
        ],
    },
)
