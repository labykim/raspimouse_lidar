import numpy as np
import math

def cart_to_pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol_to_cart(rho, phi_d):
    phi = math.radians(phi_d)
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

if __name__ == '__main__':
    pass
