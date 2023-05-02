from setuptools import setup, find_packages

package_name = 'audio_package'
src_dir = 'src'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name, src_dir],
    # packages=[package_name, 'audio_package.pocketsphinx_pubsub'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files:
        ('share/' + package_name + '/launch', ['launch/audio.launch.py']),
        # Include library files:
        ('lib/' + package_name, ['src/vocabulary.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='erik.lindby@gmail.com',
    description='Handles Keyword Spotting and other audio related features of the project.',
    license='This project is for a masters thesis by Jajep18 and Ercos18.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Include nodes:
            'pocketsphinx_pubsub = audio_package.pocketsphinx_pubsub:main',
            'mic_sensor = audio_package.mic_sensor_node:main'
        ],
    },
)
