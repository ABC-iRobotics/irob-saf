from setuptools import setup

package_name = 'irob_utils'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tamas Levendovics',
    maintainer_email='tamas.levendovics@irob.uni-obuda.hu',
    description='Python utility functions for irob-saf',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
