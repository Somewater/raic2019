from typing import Callable, Tuple, Any

from model import *
from vector import *
from predefined import *
import random
import time

def clamp_vector(vector: Vector3D, value: float):
    if vector.len() <= value:
        return vector
    else:
        return vector.normalize() * value

class GameState:
    __slots__ = 'my_score', 'enemy_score'
    def __init__(self, game: Game):
        for p in game.players:
            if p.me:
                self.my_score = p.score
            else:
                self.enemy_score = p.score

class Entity:
    __slots__ = 'position', 'radius', 'mass', 'velocity', 'radius_change_speed'

    def __init__(self,
                 position: Vector3D = None,
                 radius: float = 1.0,
                 mass: float = 1.0,
                 velocity: Vector3D = None,
                 radius_change_speed: float = 0.0):
        self.position: Vector3D = position or Vector3D()
        self.radius = radius
        self.mass = mass
        self.velocity: Vector3D = velocity or Vector3D()
        self.radius_change_speed = radius_change_speed

class RobotEntity(Entity):
    __slots__ = 'touch', 'touch_normal', 'nitro_amount', 'action', 'id', 'is_teammate'

    @staticmethod
    def from_robot(robot: Robot, action: Action, rules: Rules):
        return RobotEntity(position=Vector3D(robot.x, robot.y, robot.z),
                           radius=robot.radius,
                           mass=rules.ROBOT_MASS,
                           velocity=Vector3D(robot.velocity_x, robot.velocity_y, robot.velocity_z),
                           radius_change_speed=action.jump_speed,
                           touch=robot.touch,
                           touch_normal= Vector3D(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z) if robot.touch else None,
                           id=robot.id,
                           is_teammate=robot.is_teammate,
                           action=action)

    def __init__(self,
                 position: Vector3D = None,
                 radius: float = 1.0,
                 mass: float = 1.0,
                 velocity: Vector3D = None,
                 radius_change_speed: float = 0.0,
                 touch: bool = False,
                 touch_normal: Vector3D = None,
                 nitro_amount: float = 0.0,
                 id: int = 0,
                 is_teammate: bool = False,
                 action: Action = None):
        super().__init__(position, radius, mass, velocity, radius_change_speed)
        self.touch: bool = touch
        self.touch_normal: Vector3D = touch_normal
        self.nitro_amount: float = nitro_amount
        self.id = id
        self.is_teammate = is_teammate
        self.action = action

class BallEntity(Entity):
    @staticmethod
    def from_ball(ball: Ball, rules: Rules):
        return BallEntity(position=Vector3D(ball.x, ball.y, ball.z),
                          radius=ball.radius,
                          mass=rules.BALL_MASS,
                          velocity=Vector3D(ball.velocity_x, ball.velocity_y, ball.velocity_z),
                          radius_change_speed=0)

class NitroEntity(Entity):
    __slots__ = 'nitro_amount', 'respawn_ticks'

    @staticmethod
    def from_nitro_pack(nitro_pack: NitroPack):
        return NitroEntity(position=Vector3D(nitro_pack.x, nitro_pack.y, nitro_pack.z),
                           radius=nitro_pack.radius,
                           mass=None,
                           velocity=None,
                           radius_change_speed=None,
                           nitro_amount=nitro_pack.nitro_amount,
                           respawn_ticks=nitro_pack.respawn_ticks)

    def __init__(self,
                 position: Vector3D = None,
                 radius: float = 1.0,
                 mass: float = 1.0,
                 velocity: Vector3D = None,
                 radius_change_speed: float = 0.0,
                 nitro_amount: int = 0,
                 respawn_ticks: int = 0):
        super().__init__(position, radius, mass, velocity, radius_change_speed)
        self.nitro_amount = nitro_amount
        self.respawn_ticks = respawn_ticks

def collide_entities(rules: Rules,
                     a: Entity,
                     b: Entity,
                     random: Callable[[float, float], float] = random.uniform):
    delta_position = b.position - a.position
    distance = delta_position.len()
    penetration = a.radius + b.radius - distance
    if penetration > 0:
        k_a = (1 / a.mass) / ((1 / a.mass) + (1 / b.mass))
        k_b = (1 / b.mass) / ((1 / a.mass) + (1 / b.mass))
        normal = delta_position.normalize()
        a.position -= normal * penetration * k_a
        b.position += normal * penetration * k_b
        delta_velocity = normal.dot(b.velocity - a.velocity) + b.radius_change_speed - a.radius_change_speed
        if delta_velocity < 0:
            impulse = normal * (1 + random(rules.MIN_HIT_E, rules.MAX_HIT_E)) * delta_velocity
            a.velocity += impulse * k_a
            b.velocity -= impulse * k_b

def collide_with_arena(rules: Rules, e: Entity):
    distance, normal = dan_to_arena(rules.arena, e.position)
    penetration = e.radius - distance
    if penetration > 0:
        e.position += normal * penetration
        velocity = e.velocity.dot(normal) - e.radius_change_speed
        if velocity < 0:
            arena_e = None
            if isinstance(e, RobotEntity):
                arena_e = rules.ROBOT_ARENA_E
            else:
                arena_e = rules.BALL_ARENA_E
            e.velocity -= normal * (1 + arena_e) * velocity
            return normal
    return None

def move(rules: Rules, e: Entity, delta_time: float):
    e.velocity = clamp_vector(e.velocity, rules.MAX_ENTITY_SPEED)
    e.position += e.velocity * delta_time
    e.position.set_y(e.position.get_y() - rules.GRAVITY * delta_time * delta_time / 2)
    e.velocity.set_y(e.velocity.get_y() - rules.GRAVITY * delta_time)

def update(rules: Rules,
           delta_time: float,
           robots: List[RobotEntity],
           ball: BallEntity,
           nitros: List[NitroEntity],
           game_state: GameState):
    random.shuffle(robots[:])
    for robot in robots:
        target_velocity = Vector3D(robot.action.target_velocity_x, robot.action.target_velocity_y, robot.action.target_velocity_z)
        if robot.touch:
            target_velocity = target_velocity.min(rules.ROBOT_MAX_GROUND_SPEED)
            target_velocity -= robot.touch_normal * robot.touch_normal.dot(target_velocity)
            target_velocity_change = target_velocity - robot.velocity
            if target_velocity_change.len() > 0:
                acceleration = rules.ROBOT_ACCELERATION * max(0, robot.touch_normal.get_y())
                robot.velocity += (target_velocity_change.normalize() * acceleration * delta_time).min(target_velocity_change.len())
        if robot.action.use_nitro:
            target_velocity_change = (target_velocity - robot.velocity).min(robot.nitro_amount * rules.NITRO_POINT_VELOCITY_CHANGE)
            if target_velocity_change.len() > 0:
                acceleration = target_velocity_change.normalize() * rules.ROBOT_NITRO_ACCELERATION
                velocity_change = (acceleration * delta_time).min(target_velocity_change.len())
                robot.velocity += velocity_change
                robot.nitro_amount -= velocity_change.len() / rules.NITRO_POINT_VELOCITY_CHANGE

        move(rules, robot, delta_time)
        robot.radius = rules.ROBOT_MIN_RADIUS + (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed / rules.ROBOT_MAX_JUMP_SPEED
        robot.radius_change_speed = robot.action.jump_speed

    move(rules, ball, delta_time)

    for i in range(len(robots)):
        for j in range(i):
            collide_entities(rules, robots[i], robots[j], random=lambda x,y: min(x, y) + abs(x - y) / 2)

    for robot in robots:
        collide_entities(rules, robot, ball)
        collision_normal = collide_with_arena(rules, robot)
        if collision_normal is None:
            robot.touch = False
        else:
            robot.touch = True
            robot.touch_normal = collision_normal
    collide_with_arena(rules, ball)
    if abs(ball.position.get_z()) > rules.arena.depth / 2 + ball.radius:
        if ball.position.get_z() > 0: # TODO: check it
            game_state.my_score += 1
        else:
            game_state.enemy_score += 1

    for robot in robots:
        if robot.nitro_amount == rules.MAX_NITRO_AMOUNT:
            continue
        for pack in nitros:
            if pack.nitro_amount == 0:
                continue
            if (robot.position - pack.position).len() <= robot.radius + pack.radius:
                robot.nitro_amount = rules.MAX_NITRO_AMOUNT
                pack.nitro_amount = 0
                pack.respawn_ticks = rules.NITRO_PACK_RESPAWN_TICKS

def tick(rules: Rules,
         robots: List[RobotEntity],
         ball: BallEntity,
         nitros: List[NitroEntity],
         game_state: GameState):
    delta_time = 1 / rules.TICKS_PER_SECOND
    for _ in range(rules.MICROTICKS_PER_TICK):
        update(rules, delta_time / rules.MICROTICKS_PER_TICK, robots, ball, nitros, game_state)
    for pack in nitros:
        if pack.nitro_amount > 0:
            continue
        pack.respawn_ticks -= 1
        if pack.respawn_ticks == 0:
            pack.nitro_amount = rules.NITRO_PACK_AMOUNT

class Engine:
    def __init__(self, me: Robot, rules: Rules, game: Game):
        self.rules = rules
        self.robot_entities = []
        for r in game.robots:
            action = Action()
            self.robot_entities.append(RobotEntity.from_robot(r, action, rules))
        self.ball_entity = BallEntity.from_ball(game.ball, rules)
        self.nitro_entities = []
        for n in game.nitro_packs:
            self.nitro_entities.append(NitroEntity.from_nitro_pack(n))
        self.game_state = GameState(game)

    def tick(self):
        start = time.time()
        tick(self.rules, self.robot_entities, self.ball_entity, self.nitro_entities, self.game_state)
        print('tick in %.2f ms' % (1000 * (time.time() - start)))