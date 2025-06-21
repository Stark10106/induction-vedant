from setuptools import setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stark',
    maintainer_email='your_email@example.com',  # Optional: replace with yours
    description='Publisher node to control TurtleBot3 with geometry_msgs/Twist.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller = turtle_controller.publisher:main',
        ],
    },
)

