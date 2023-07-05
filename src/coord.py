import numpy as np

def cart_to_pol(x, y):
    """
    Convert a cartesian vector to a polar vector.

    Params
        `(x, y)` Cartesian vector.

    Returns
        Polar vector `(rho, phi)`, where `phi` is an angle in degrees.
    """
    rho = np.round(np.sqrt(x**2 + y**2), 4)
    phi = np.round(np.arctan2(y, x), 4)
    return (rho, np.degrees(phi))

def pol_to_cart(rho, phi_d):
    """
    Convert a polar vector to a cartesian vector.

    Params
        `(rho, phi_d)` Polar vector where `phi_d` is an angle in degress.

    Returns
        Cartesian vector `(x, y)`.
    """
    phi = np.radians(phi_d)
    x = np.round(rho * np.cos(phi), 4)
    y = np.round(rho * np.sin(phi), 4)
    return (x, y)

def polar_velocity(p1, p2, origin):
        """
        Calculate the velocity of from p1 to p2 in Polar coordinate.
        In other words, vector substitution: p2 - p1

        Params
            p1, p2 : Points in the polar coordinate. In other words, two polar vectors.
            origin : Velocity in the ploar coordinate representing the movement of the origin when measuring p1 and p2.
        Returns
            (float) Cartesian vector (x, y) from p1 to p2
        """
        c1 = pol_to_cart(p1[0], p2[1])
        c2 = pol_to_cart(p2[0], p2[1])

        vector = [0, 0]
        vector[0] = c2[0] + origin[0] * 0.2 - c1[0]
        vector[1] = c2[1] + origin[1] * 0.2 - c1[1]
        return vector

if __name__ == '__main__':
    p1 = [4, 60]
    p2 = [2, 0]
    o = [0, 1]
    a = polar_velocity(p1, p2, o)
    print(a)
