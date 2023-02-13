from setuptools import setup

package_name = 'detection'

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
    maintainer='scg1224',
    maintainer_email='caos1@rose-hulman.edu',
    description='YoloV7 speed sign detection runs in a ros node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_ros = detection.detect_ros:main'
        ],
    },
)
