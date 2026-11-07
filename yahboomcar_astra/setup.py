from setuptools import setup, find_packages
import os
from glob import glob
package_name = 'yahboomcar_astra'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  
    #packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'config'),glob(os.path.join('config','*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='13377528435@sina.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'colorTracker = yahboomcar_astra.colorTracker:main',
        'qrTracker = yahboomcar_astra.qrTracker:main',
        'monoTracker = yahboomcar_astra.monoTracker:main',
        'faceTracker = yahboomcar_astra.faceTracker:main',
        'apriltagTracker = yahboomcar_astra.apriltagTracker:main',
        'gestureTracker = yahboomcar_astra.gestureTracker:main',
        'poseTracker = yahboomcar_astra.poseTracker:main',
        'colorFollow = yahboomcar_astra.colorFollow:main',
        'faceFollow = yahboomcar_astra.faceFollow:main',
        'qrFollow = yahboomcar_astra.qrFollow:main',
        'monoFollow = yahboomcar_astra.monoFollow:main',
        'gestureFollow = yahboomcar_astra.gestureFollow:main',
        'poseFollow = yahboomcar_astra.poseFollow:main',
        'apriltagFollow = yahboomcar_astra.apriltagFollow:main',
        'follow_line = yahboomcar_astra.follow_line:main',
        'HandCtrl = yahboomcar_astra.HandCtrl:main',
        'qrCtrl = yahboomcar_astra.qrCtrl:main',
        ],
    },
)
