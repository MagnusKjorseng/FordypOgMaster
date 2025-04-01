from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name, ['config.yml']),
        #(os.path.join('share',package_name,'launch'), glob(os.path.join('launch', 'config.yml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muzz',
    maintainer_email='magnurkj@stud.ntnu.no',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv = controller.usv_controller:main',
            'alloc = controller.usv_allocator:main',
            'translator = controller.sim_translator:main',
        ],
    },
)
