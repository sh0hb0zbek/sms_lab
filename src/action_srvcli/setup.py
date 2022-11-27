from setuptools import setup
import os
from glob import glob

package_name = 'action_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shokhbozbek',
    maintainer_email='shkhalimjonov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server = action_srvcli.fibonacci_action_server:main',
            'action_client = action_srvcli.fibonacci_action_client:main',
        ],
    },
)
