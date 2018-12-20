from .player import *
from .ball import *
from .nitro_pack import *
from .robot import *
from typing import List

class Game:
    def __init__(self, json):
        self.current_tick: int = json.get("current_tick")
        self.players: List[Player] = list(map(Player, json.get("players")))
        self.robots: List[Robot] = list(map(Robot, json.get("robots")))
        self.nitro_packs: List[NitroPack] = list(map(NitroPack, json.get("nitro_packs")))
        self.ball: Ball = Ball(json.get("ball"))
