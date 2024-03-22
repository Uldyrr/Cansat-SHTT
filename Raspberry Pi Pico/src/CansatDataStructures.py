class Vector3:
    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0

    def __init__(self, x: float, y: float, z: float):
        self.X = x
        self.Y = y
        self.Z = z

    def __str__(self):
        return f"{self.X}, {self.Y}, {self.Z}"

    @staticmethod
    def Empty():
        return Vector3(0, 0, 0)