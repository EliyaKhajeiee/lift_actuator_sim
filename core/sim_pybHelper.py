import pybullet as p

def _q(roll=0.0, pitch=0.0, yaw=0.0):
    return p.getQuaternionFromEuler([roll, pitch, yaw])


def _box(half, pos, orn=None, color=None, mass=0):
    orn = orn or _q()
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = (p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(mass, col, vis, pos, orn)


def _cyl(r, h, pos, orn=None, color=None, mass=0):
    orn = orn or _q()
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
    vis = (p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(mass, col, vis, pos, orn)


def _sph(r, pos, color=None):
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=r)
    vis = (p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0, col, vis, pos)