import numpy as np

def cart_to_pol(vector: list) -> list:
    """
    Convert a cartesian vector to a polar vector.

    ### Parameters
        Cartesian vector `vector = [x, y]`.

    ### Returns
        Polar vector `[rho, phi]`, where `phi` is an angle in degrees.
    """
    x = vector[0]
    y = vector[1]

    rho = np.round(np.sqrt(x**2 + y**2), 6)
    phi = np.round(np.arctan2(y, x), 6)
    return [rho, np.degrees(phi)]

def pol_to_cart(vector: list) -> list:
    """
    Convert a polar vector to a cartesian vector.

    ### Params
        Polar vector `vector = [rho, phi]`, where `phi` is an angle in degrees.

    ### Returns
        Cartesian vector `[x, y]`.
    """
    rho = vector[0]
    phi = np.radians(vector[1])

    x = rho * np.cos(phi)
    y = rho * np.sin(phi)

    return [x, y]

def polar_velocity(p1: list, p2: list, po: list) -> list:
    """
    Calculate the velocity of an object from p1 to p2 in the polar coordinate system, considering the observer (i.e., origin) moves. In other words, calculate `p2 + po - p1`.\n
    An object at p1 is observed on t1 and p2 on p2, while the observer may be moved. t2 is the current updating time and t1 is the last updating time; mostly t2 = t1 + 0.2 as the RPM of the light sensor (LDS-01) is 300.

    ### Params
        `p1`, `p2` : Polar vector.
        `po` : Velocity of the origin (i.e., observer) when t1. 
        
    ### Returns
        Cartesian vector `[x, y]`
    """
    c1 = pol_to_cart(p1)
    c2 = pol_to_cart(p2)
    co = pol_to_cart(po)

    # print('c1: ' , c1)
    # print('c2: ' , c2)
    # print('o : ' , o)

    vector = [0, 0]
    vector[0] = c2[0] + co[0] * 0.2 - c1[0]
    vector[1] = c2[1] + co[1] * 0.2 - c1[1]

    return vector

if __name__ == '__main__':
    p1 = [4, 150]
    p2 = [4, 30]
    o = [0, 0]

    a = polar_velocity(p1, p2, o)
    a = cart_to_pol(a)
    print(a)
