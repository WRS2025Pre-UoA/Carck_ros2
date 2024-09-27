from setuptools import setup

package_name = 'crack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ruiiwata837@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = crack.test_publisher:main',
            'image_publisher = crack.image_publisher:main',  # パッケージ名を小文字に変更
            #'image_subscriber = crack.image_subscriber:main',  # パッケージ名を小文字に変更
            'crack_subscriber = crack.crack_subscriber:main',  # パッケージ名を小文字に変更
        ],
    },
)
