from setuptools import setup

package_name = 'python_plot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adi99',
    maintainer_email='meduri99aditya@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot = python_plot_pkg.plot_path:main',
            'plot_client = python_plot_pkg.plot_path_cli:main',
            'var_speed = python_plot_pkg.plot_var_speed:main',
            'curved_path = python_plot_pkg.plot_curved_path:main',
            'circle_path = python_plot_pkg.plot_circle_path:main',
            'plot_carla = python_plot_pkg.plot_carla:main',
            'plot_intersection = python_plot_pkg.plot_intersection:main',
            'plot_inter_wide_lane = python_plot_pkg.plot_inter_wide_lane:main',
            'plot_lane_merge = python_plot_pkg.plot_lane_merge:main',
            'plot_idm = python_plot_pkg.plot_idm:main',
        ],
    },
)
