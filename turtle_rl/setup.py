from pathlib import Path

from setuptools import find_packages, setup


def recursive_files(prefix, path):
    """
    Recurse over path returning a list of tuples suitable for use with setuptools data_files.

    param prefix : prefix path to prepend to the path
    param path : Path to directory to recurse. Path should not have a trailing '/'
    return : List of tuples.
        First element of each tuple is destination path.
        Second element is a list of files to copy to that path.
    """
    file_list = []
    for subdir in Path(path).glob('**'):
        file_list.append((str(Path(prefix)/subdir),
                          [str(file) for file in subdir.glob('*') if not file.is_dir()]))
    return file_list


package_name = 'turtle_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/env-hooks', ['env-hooks/turtle_rl.dsv']),
        ('share/' + package_name, ['config/turtle_rviz.rviz']),
        ('share/' + package_name, ['config/gazebo_gui.config']),
        ('share/' + package_name, ['urdf/turtlebot3_burger.urdf.xacro']),
        ('share/' + package_name, ['urdf/turtlebot3_burger.gazebo.xacro']),
        ('share/' + package_name, ['launch/turtle_rviz.launch.xml']),
        ('share/' + package_name, ['launch/turtle.launch.xml']),
        ('share/' + package_name, ['launch/start_gazebo.launch.xml']),
        ('share/' + package_name, ['launch/training.launch.xml']),
        ('share/' + package_name, ['launch/retraining.launch.xml']),
        *recursive_files('share/' + package_name, 'meshes'),
        *recursive_files('share/' + package_name, 'worlds'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zixin Ye',
    maintainer_email='zixinye2026@u.northwestern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'moveTest = turtle_rl.moveTest:main',
            'simplified_environment = turtle_rl.simplified_env:main',
            'gazebo_controller = turtle_rl.gazebo_controller:main',
            'training_node = turtle_rl.training_node:main',
        ],
    },
)
