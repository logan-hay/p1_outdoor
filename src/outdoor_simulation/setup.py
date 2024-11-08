from setuptools import find_packages, setup

package_name = 'outdoor_simulation'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/outdoor_simulation_launch.py']))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/outdoor.wbt', 
]))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Logan Hay',
    maintainer_email='ljhay314@gmail.com',
    description='Outdoor simulation for cs460/560 project 1',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],    
        'console_scripts': ['outdoor_simulation = outdoor_simulation.outdoor_simulation:main']
    },
)

