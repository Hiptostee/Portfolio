from setuptools import find_packages, setup


package_name = 'slambot_mobile_bridge'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/slambot_mobile_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joseph Marra',
    maintainer_email='joey.marra2007@gmail.com',
    description='FastAPI and WebSocket bridge for the Slambot mobile product.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = slambot_mobile_bridge.bridge_node:main',
        ],
    },
)
