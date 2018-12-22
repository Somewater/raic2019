from typing import Callable, Tuple, Any

from model import *
from vector import *
from predefined import *
import random

class GameState:
    __slots__ = 'my_score', 'enemy_score'
    def __init__(self, game: Game = None):
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
    __slots__ = 'touch', 'touch_normal', 'nitro_amount'

    def __init__(self,
                 position: Vector3D = None,
                 radius: float = 1.0,
                 mass: float = 1.0,
                 velocity: Vector3D = None,
                 radius_change_speed: float = 0.0,
                 robot: Robot = None):
        super().__init__(position, radius, mass, velocity, radius_change_speed)
        if robot:
            self.touch: bool = robot.touch
            self.touch_normal: Vector3D = Vector3D(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z)
            self.nitro_amount: float = robot.nitro_amount
        else:
            self.touch = False
            self.touch_normal = None
            self.nitro_amount = 0.0

class BallEntity(Entity):
    pass

class NitroEntity(Entity):
    __slots__ = 'alive', 'respawn_ticks'

    def __init__(self,
                 position: Vector3D = None,
                 radius: float = 1.0,
                 mass: float = 1.0,
                 velocity: Vector3D = None,
                 radius_change_speed: float = 0.0,
                 nitro_pack: NitroPack = None):
        super().__init__(position, radius, mass, velocity, radius_change_speed)
        if nitro_pack:
            self.alive = nitro_pack.nitro_amount > 0
            self.respawn_ticks = nitro_pack.respawn_ticks
        else:
            self.alive = False
            self.respawn_ticks = 0

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
            impulse = (1 + random(rules.MIN_HIT_E, rules.MAX_HIT_E)) * delta_velocity * normal
            a.velocity += impulse * k_a
            b.velocity -= impulse * k_b

def collide_with_arena(arena: Arena, e: Entity):
    distance, normal = dan_to_arena(arena, e.position)
    penetration = e.radius - distance
    if penetration > 0:
        e.position += penetration * normal
        velocity = e.velocity.dot(normal) - e.radius_change_speed
        if velocity < 0:
            e.velocity -= (1 + e.arena_e) * velocity * normal
            return normal
    return None

def move(rules: Rules, e: Entity, delta_time: float):
    e.velocity = min(e.velocity, rules.MAX_ENTITY_SPEED)
    e.position += e.velocity * delta_time
    e.position.y -= rules.GRAVITY * delta_time * delta_time / 2
    e.velocity.y -= rules.GRAVITY * delta_time

def update(rules: Rules,
           delta_time: float,
           robots: List[Tuple[RobotEntity, Action]],
           ball: BallEntity,
           arena: Arena,
           nitros: List[NitroEntity],
           game_state: GameState):
    random.shuffle(robots[:])
    for robot, robot_action in robots:
        target_velocity = Vector3D(robot_action.target_velocity_x, robot_action.target_velocity_y, robot_action.target_velocity_z)
        if robot.touch:
            target_velocity = target_velocity.min(rules.ROBOT_MAX_GROUND_SPEED)
            target_velocity -= robot.touch_normal * robot.touch_normal.dot(target_velocity)
            target_velocity_change = target_velocity - robot.velocity
            if target_velocity_change.len() > 0:
                acceleration = rules.ROBOT_ACCELERATION * max(0, robot.touch_normal.y)
                robot.velocity += (target_velocity_change.normalize() * acceleration * delta_time).min(target_velocity_change.len())
        if robot_action.use_nitro:
            target_velocity_change = (target_velocity - robot.velocity).min(robot.nitro_amount * rules.NITRO_POINT_VELOCITY_CHANGE)
            if target_velocity_change.len() > 0:
                acceleration = target_velocity_change.normalize() * rules.ROBOT_NITRO_ACCELERATION
                velocity_change = (acceleration * delta_time).min(target_velocity_change.len())
                robot.velocity += velocity_change
                robot.nitro_amount -= velocity_change.len() / rules.NITRO_POINT_VELOCITY_CHANGE # TODO: don't touch model

        move(rules, robot, delta_time)
        robot.radius = rules.ROBOT_MIN_RADIUS + (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * robot_action.jump_speed / rules.ROBOT_MAX_JUMP_SPEED
        robot.radius_change_speed = robot_action.jump_speed

    move(ball)

    for i in range(len(robots)):
        for j in range(i):
            collide_entities(robots[i], robots[j])

    for robot in robots:
        collide_entities(robot, ball)
        collision_normal = collide_with_arena(arena, robot)
        if collision_normal is None:
            robot.touch = False
        else:
            robot.touch = True
            robot.touch_normal = collision_normal
    collide_with_arena(arena, ball)
    if abs(ball.position.z) > arena.depth / 2 + ball.radius:
        if ball.position.z > 0: # TODO: check it
            game_state.my_score += 1
        else:
            game_state.enemy_score += 1

    for robot, _ in robots:
        if robot.nitro_amount == rules.MAX_NITRO_AMOUNT:
            continue
        for pack in nitros:
            if not pack.alive:
                continue
            if (robot.position - pack.position).len() <= robot.radius + pack.radius:
                robot.nitro_amount = rules.MAX_NITRO_AMOUNT
                pack.alive = False
                pack.respawn_ticks = rules.NITRO_PACK_RESPAWN_TICKS

def tick(rules: Rules,
         robots: List[Tuple[RobotEntity, Action]],
         ball: BallEntity,
         arena: Arena,
         nitros: List[NitroEntity]):
    delta_time = 1 / rules.TICKS_PER_SECOND
    for _ in range(rules.MICROTICKS_PER_TICK):
        update(rules, delta_time / rules.MICROTICKS_PER_TICK, robots, ball, arena, nitros)
    for pack in nitros:
        if pack.alive:
            continue
        pack.respawn_ticks -= 1
        if pack.respawn_ticks == 0:
            pack.alive = True

class Engine:
    def __init__(self, rules: Rules, game: Game):
        pass