#!/usr/bin/env python

"""
setup.py file for SWIG PID controller
"""

from distutils.core import setup, Extension


pid_controller_module = Extension('_pid_controller',
                           sources=['pid_controller_wrap.c', 'pid_controller.c'],
                           )

setup (name = 'example',
       version = '0.1',
       author      = "Kimberly McGuire (Bitcraze A.B.)",
       description = """Python wrapper for c-based PID controller for the crazyflie""",
       ext_modules = [pid_controller_module],
       py_modules = ["pid_controller"],
       )