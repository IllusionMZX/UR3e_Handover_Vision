from setuptools import setup

package_name = 'vision_pick_and_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[
        'hsv_pick_and_place_eye_in_hand',
        'hsv_real_time_tracking_cube_eye_in_hand',
        'hsv_real_time_tracking_cube_eye_in_hand_6d'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miao Zixiang',
    maintainer_email='miao.zixiang@foxmail.com',
    description='Vision Pick and Place Node for UR3e',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hsv_pick_and_place = hsv_pick_and_place_eye_in_hand:main',
            'hsv_tracking = hsv_real_time_tracking_cube_eye_in_hand:main',
            'hsv_tracking_6d = hsv_real_time_tracking_cube_eye_in_hand_6d:main',
        ],
    },
)
