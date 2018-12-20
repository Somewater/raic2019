class Player:
    def __init__(self, json):
        self.id: int = json.get("id")
        self.me: bool = json.get("me")
        self.strategy_crashed: bool = json.get("strategy_crashed")
        self.score: int = json.get("score")
