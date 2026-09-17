from setuptools import find_packages, setup

package_name = 'pcl_clustering_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimkh',
    maintainer_email='sbzmf1@o.cnu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],# setup.py 파일의 entry_points 딕셔너리를 수정
    entry_points={
        'console_scripts': [
            'euclidean_cluster_node = pcl_clustering_py.euclidean_cluster_node:main',
        ],
    },
)
