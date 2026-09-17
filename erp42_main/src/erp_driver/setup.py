from setuptools import setup
import os
from glob import glob

package_name = 'erp_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'erp_driver.ByteHandler',
        'erp_driver.ErpSerialHandler',
        'erp_driver.erp42_serial',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ERP-42 ROS2 Driver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "erp_base = erp_driver.erp42_serial:main",
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)

