from setuptools import find_packages, setup
import os
import glob

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' +package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv_python',
        'Pillow'
        ],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='hjs4724@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = vision_pkg.yolo_node:main'
        ],
    },
)
