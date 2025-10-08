import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'model_sentra'

# Archivos URDF y XACRO en urdf/
urdf_files = glob('urdf/*.urdf') + glob('urdf/*.xacro')

# Archivos STL (minúsculas y mayúsculas) dentro de urdf/stl
stl_files = glob('urdf/stl/*.stl') + glob('urdf/stl/*.STL')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

if urdf_files:
    data_files.append((os.path.join('share', package_name, 'urdf'), urdf_files))

if stl_files:
    data_files.append((os.path.join('share', package_name, 'urdf', 'stl'), stl_files))

# Archivos de launch
launch_files = glob('launch/*.py')
if launch_files:
    data_files.append((os.path.join('share', package_name, 'launch'), launch_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='johann',
    maintainer_email='johann.rodriguezv@gmail.com',
    description='Modelo Sentra para simulación',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
