from setuptools import setup

package_name = 'smart_g'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['inicio.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan',
    maintainer_email='juan_p.rivera@uao.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'VideoInput = smart_g.VideoInput:main',
        'image_filterA = smart_g.image_filterA:main',
        'detector = smart_g.detector:main',
        'image_filterSubs = smart_g.image_filterSubs:main',
        'obj_Tracker = smart_g.tracker:main',
        'leaf_define = smart_g.leaf_define:main',
        'outputNode = smart_g.outputNode:main',
        ],
    },
)
