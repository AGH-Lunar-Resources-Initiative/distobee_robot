from setuptools import setup

package_name = 'distobee_hardware'

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
    maintainer='RafalS3',
    maintainer_email='102436271+RafalS3@users.noreply.github.com',
    description='distobee_hardware',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_driver = distobee_hardware.wheel_driver_node:main',
        ],
    },
)
