from setuptools import find_packages, setup

package_name = 'Robot'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'image_capturer = Robot.image_capturer:main', 
             'color_test = Robot.color_test:main',
             'safe_test = Robot.safe_test:main',
             'test = Robot.test:main',
             'basetest = Robot.basetest:main',
             'rectantest = Robot.rectantest:main',
        ],
    },
)
