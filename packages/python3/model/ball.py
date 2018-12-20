class Ball:
    def __init__(self, json):
        self.x: float = json.get("x")
        self.y: float = json.get("y")
        self.z: float = json.get("z")
        self.velocity_x: float = json.get("velocity_x")
        self.velocity_y: float = json.get("velocity_y")
        self.velocity_z: float = json.get("velocity_z")
        self.radius: float = json.get("radius")
