from setuptools import setup

package_name = 'uwb_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/static_tf.launch.py',
            'launch/static_th.launch.py',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/uwb_viz.rviz']),
        ('share/' + package_name + '/config', ['config/anchors.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilan',
    maintainer_email='ilan.refael@mapcore.com',
    description='UWB radius visualization in RViz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'radius_viz = uwb_viz.radius_viz_node:main',
        ],
    },
)
