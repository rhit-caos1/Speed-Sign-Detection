from setuptools import setup

package_name = 'speed_detect'

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
    description='Reading speed info from a LED 7 seg number screen',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speed = speed_detect.speed:main'
        ],
    },
)
