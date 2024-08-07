from setuptools import find_packages, setup

package_name = 'image_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bushwookie',
    maintainer_email='bushwookie@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_location_service = image_processing.arm_location_service:main',
            'legacy_arm_location = image_processing.legacy_arm_location:main',
            'zed_location_service = image_processing.zed_location_service:main'
        ],
    },
)
