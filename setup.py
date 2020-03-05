from glob import glob
from setuptools import setup, Extension

setup(
  name='c_ext',
  ext_modules=[
    Extension(
      'c_ext',
      glob('c_ext/**/*.c*', recursive=True),
      include_dirs=['lib/pybind11/include'],
      extra_compile_args=['-std=c++11'],
    ),
  ],
)
