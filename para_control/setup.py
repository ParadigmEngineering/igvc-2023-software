from setuptools import setup
from glob import glob

package_name = 'para_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + f'/{package_name}/', glob(f'{package_name}/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paradigm',
    maintainer_email='anash@mun.ca',
    description='Run Paradigm AI Model',
    license='Beerware v42',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'para_control = para_control.keyboard_control_node:main'
        ],
    },
)
