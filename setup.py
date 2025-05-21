from setuptools import setup, Extension
import pybind11

pathfinder_module = Extension(
    'pathfinder',
    sources=['pathfinder.cpp', 'pathfinder_bindings.cpp'],
    include_dirs=[pybind11.get_include()],
    language='c++',
    extra_compile_args=['-std=c++17', '-O3'],  # Enable optimizations
)

setup(
    name='pathfinder',
    version='1.0',
    description='Theta* pathfinding implementation with Python bindings',
    ext_modules=[pathfinder_module],
    python_requires='>=3.7',
)