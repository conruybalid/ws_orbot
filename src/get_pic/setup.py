from setuptools import find_packages, setup

package_name = 'get_pic'

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
    maintainer='me',
    maintainer_email='me@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = get_pic.videoPublisher:main',
            'video_subscriber = get_pic.videoSubscriber:main',
            'zed_subscriber = get_pic.zed_subscriber:main',
            'zed_publisher = get_pic.zed_publisher:main',
        ],
    },
)
