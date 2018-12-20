class NitroPack:
    def __init__(self, json):
        self.id: int = json.get("id")
        self.x: float = json.get("x")
        self.y: float = json.get("y")
        self.z: float = json.get("z")
        self.radius: float = json.get("radius")
        self.nitro_amount = json.get("nitro_amount")
        self.respawn_ticks = json.get("respawn_ticks")
