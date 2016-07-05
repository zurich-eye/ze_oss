#!/usr/bin/python
"""
Zurich Eye

This script requires the installation of Sympy:
 > sudo apt-get install python3-pip
 > sudo pip3 install sympy
"""

import sympy as sy

a = sy.Symbol('a')
b = sy.Symbol('b')
c = sy.Symbol('c')
d = sy.Symbol('d')

# Pixel
x = sy.Symbol('x')
y = sy.Symbol('y')
u = sy.Matrix([[x],
               [y]])
               
# Jacobian
E = sy.Matrix([[a, b],
               [d, c]])
       
E2 = E.transpose() * E     
E3 = E2.inv() * E.transpose()

u2 = E3 * u