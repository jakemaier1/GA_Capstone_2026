from setuptools import find_packages, setup

package_name = 'yahboomcar_depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/camera_app.launch.py',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_colorTracker = yahboomcar_depth.Advanced.colorTracker:main',
            'depth_to_color = yahboomcar_depth.Basic.depth_to_color:main',
            'get_center_dis = yahboomcar_depth.Basic.get_center_dis:main',
            'calculate_volume = yahboomcar_depth.Basic.calculate_volume:main',
            'depth_faceFollow = yahboomcar_depth.Advanced.faceFollow:main',
            'depth_qrFollow = yahboomcar_depth.Advanced.qrFollow:main',
            'depth_apriltagFollow = yahboomcar_depth.Advanced.apriltagFollow:main',
            'KCF_Tracker = yahboomcar_depth.kcf.KCF_Tracker:main',
            'depth_gestureFollow = yahboomcar_depth.MediaPipe.gestureFollow:main',
            'depth_poseFollow = yahboomcar_depth.MediaPipe.poseFollow:main',
            'depth_follow_line = yahboomcar_depth.Advanced.follow_line:main',
            'camera_app = yahboomcar_depth.camera_app:main',
            'Edge_Detection = yahboomcar_depth.Advanced.Edge_Detection:main',
        ],
    },
)

