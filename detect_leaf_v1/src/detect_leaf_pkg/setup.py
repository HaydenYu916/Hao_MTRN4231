from setuptools import find_packages, setup

package_name = 'detect_leaf_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/leaf_detector.launch.py']),
    ],
    install_requires=['setuptools', 'plantcv', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='hao',
    maintainer_email='hao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'leaf_detector = detect_leaf_pkg.leaf_detector:main',
            'coordinates_display = detect_leaf_pkg.coordinates_display:main',
        ],
    },
)
