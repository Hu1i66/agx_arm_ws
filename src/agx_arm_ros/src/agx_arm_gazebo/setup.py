from setuptools import setup
import os
from glob import glob

package_name = 'agx_arm_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装模型/配置文件（如果有）
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex',
    maintainer_email='support@agilex.ai',
    description='PiPER mechanical arm gazebo simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 核心：将joint8_ctrl.py注册为可执行节点 joint8_ctrl
            'joint8_ctrl = agx_arm_gazebo.joint8_ctrl:main'
        ],
    },
)
