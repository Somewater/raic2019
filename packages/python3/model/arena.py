class Arena:
    def __init__(self, json):
        self.width: float = json.get("width")
        self.height: float = json.get("height")
        self.depth: float = json.get("depth")
        self.bottom_radius: float = json.get("bottom_radius")
        self.top_radius: float = json.get("top_radius")
        self.corner_radius: float = json.get("corner_radius")
        self.goal_top_radius: float = json.get("goal_top_radius")
        self.goal_width: float = json.get("goal_width")
        self.goal_height: float = json.get("goal_height")
        self.goal_depth: float = json.get("goal_depth")
        self.goal_side_radius: float = json.get("goal_side_radius")
