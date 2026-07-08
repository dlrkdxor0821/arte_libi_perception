from setuptools import find_packages, setup

package_name = 'follower_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py_trees', 'transitions', 'numpy'],
    zip_safe=True,
    maintainer='leekt',
    maintainer_email='dlrkdxor0821@gmail.com',
    description='Vision-based person follow control.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_node = follower_control.control_node:main',
        ],
    },
)
