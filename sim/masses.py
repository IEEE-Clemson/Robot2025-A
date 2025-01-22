import numpy as np

class Mass:
    def __init__(self):
        pass

    def get_moment_2d(self) -> float:
        return 0
    
    def get_mass(self) -> float:
        return 0
    
class SquareMass(Mass):
    def __init__(self, pos: np.ndarray, size: np.ndarray, mass: float):
        super().__init__()
        self.pos = np.array(pos)
        self.size = np.array(size)
        self.mass = mass
    

    def get_moment_2d(self) -> float:
        m = self.mass
        x = self.pos[0]
        y = self.pos[1]
        h = self.size[0]
        w = self.size[0]
        return m*(x**2 + y**2 + h**2/12 + w**2/12)
    
    def get_mass(self) -> float:
        return self.mass