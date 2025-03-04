from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext as _build_ext
from Cython.Build import cythonize


class BuildAvocadoExt(_build_ext):
    """Builds AVOCADO before our module."""

    def run(self):
        # Build AVOCADO
        import os
        import os.path
        import subprocess

        build_dir = os.path.abspath('build/AVOCADO')
        if not os.path.exists(build_dir):
            os.makedirs(build_dir)
            subprocess.check_call(['cmake', '../..', '-DCMAKE_CXX_FLAGS=-fPIC'],
                                  cwd=build_dir)
        subprocess.check_call(['cmake', '--build', '.'], cwd=build_dir)

        _build_ext.run(self)


extensions = [
    Extension('avocado', ['src/*.pyx'],
              include_dirs=['src'],
              libraries=['AVOCADO'],
              library_dirs=['build/AVOCADO/src'],
              extra_compile_args=['-fPIC']),
]

setup(
    name="pyavocado",
    ext_modules=cythonize(extensions),
    cmdclass={'build_ext': BuildAvocadoExt},
    classifiers=[
        # 'Development Status :: 5 - Production/Stable',
        # 'Intended Audience :: Developers',
        # 'Intended Audience :: Education',
        # 'Intended Audience :: Information Technology',
        # 'Operating System :: OS Independent',
        # 'Programming Language :: Python',
        # 'Programming Language :: Python :: 2.7',
        # 'Programming Language :: Python :: 3',
        # 'Programming Language :: Python :: 3.4',
        # 'Programming Language :: Cython',
        # 'Topic :: Games/Entertainment :: Simulation',
        # 'Topic :: Software Development :: Libraries :: Python Modules',
    ],
)
