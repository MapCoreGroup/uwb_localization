from setuptools import find_packages, setup

package_name = 'uwb_serial_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    
        ('share/' + package_name + '/launch',
         ['launch/uwb_characterization.launch.py',
          'launch/multi_anchor_circles.launch.py']),
        ('share/' + package_name + '/rviz',
         ['rviz/multi_anchor_circles.rviz']),
        ('share/' + package_name + '/urdf',
         ['urdf/anchors.urdf'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilan',
    maintainer_email='ilan@todo.todo',
    description='UWB anchor serial reader nodes',
    license='MIT',
    tests_require=['pytest'],

   
    entry_points={
        'console_scripts': [
            'anchor1 = uwb_serial_pub.serial_reader_anchor1:main',
            'anchor2 = uwb_serial_pub.serial_reader_anchor2:main',
            'anchor3 = uwb_serial_pub.serial_reader_anchor3:main',
            'anchor4 = uwb_serial_pub.serial_reader_anchor4:main',
            'float_to_marker = uwb_serial_pub.FloatToMarker:main',
            'anchor1_viz = uwb_serial_pub.anchor1_viz:main',
            'multi_anchor_circles = uwb_serial_pub.multi_anchor_circles:main',
            'range_stats_node = uwb_serial_pub.range_stats_node:main',
            'uwb_bayes_direction = uwb_serial_pub.uwb_bayes_direction_node:main',
            'uwb_pf_node = uwb_serial_pub.uwb_pf_node:main',
            'uwb_characterization_logger = uwb_serial_pub.uwb_characterization_logger:main'

        ],
    },
)
