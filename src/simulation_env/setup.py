from setuptools import setup

package_name = 'simulation_env'

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
            'combined_sim = simulation_env.combined_sim:main',
            'circle_sim = simulation_env.circle_env:main',
            'combined_sim_async = simulation_env.combined_sim_async:main',
        ],
    },
)
