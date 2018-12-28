from model import *
from vector_c cimport Vector3D
from predefined_c cimport dan_to_arena, Dan, ArenaStruct
import random
import time

cdef Vector3D clamp_vector(Vector3D vector, float value):
    if vector.len() <= value:
        return vector
    else:
        return vector.normalize().mul(value)

cdef class GameState:
    cdef float my_score, enemy_score

    def __cinit__(self, game: Game):
        for p in game.players:
            if p.me:
                self.my_score = p.score
            else:
                self.enemy_score = p.score

cdef struct ActionStruct:
    float target_velocity_x
    float target_velocity_y
    float target_velocity_z
    float jump_speed
    bint use_nitro

cdef struct RulesStruct:
    int max_tick_count
    ArenaStruct arena
    int team_size
    int seed
    float ROBOT_MIN_RADIUS
    float ROBOT_MAX_RADIUS
    float ROBOT_MAX_JUMP_SPEED
    float ROBOT_ACCELERATION
    float ROBOT_NITRO_ACCELERATION
    float ROBOT_MAX_GROUND_SPEED
    float ROBOT_ARENA_E
    float ROBOT_RADIUS
    float ROBOT_MASS
    float TICKS_PER_SECOND
    int MICROTICKS_PER_TICK
    int RESET_TICKS
    float BALL_ARENA_E
    float BALL_RADIUS
    float BALL_MASS
    float MIN_HIT_E
    float MAX_HIT_E
    float MAX_ENTITY_SPEED
    float MAX_NITRO_AMOUNT
    float START_NITRO_AMOUNT
    float NITRO_POINT_VELOCITY_CHANGE
    float NITRO_PACK_X
    float NITRO_PACK_Y
    float NITRO_PACK_Z
    float NITRO_PACK_RADIUS
    float NITRO_PACK_AMOUNT
    int NITRO_PACK_RESPAWN_TICKS
    float GRAVITY

cdef class Entity:
    cpdef Vector3D position
    cdef Vector3D velocity
    cdef float radius, mass, radius_change_speed

    def __init__(self,
                 Vector3D position: Vector3D,
                 float radius,
                 float mass,
                 Vector3D velocity,
                 float radius_change_speed):
        self.position = position
        self.radius = radius
        self.mass = mass
        self.velocity = velocity
        self.radius_change_speed = radius_change_speed

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity

cdef class RobotEntity:
    cpdef Entity entity
    cdef bint touch
    cpdef bint is_teammate
    cdef Vector3D touch_normal
    cdef float nitro_amount
    cdef ActionStruct action
    cpdef int id

    def __init__(self,
                 Vector3D position,
                 float radius,
                 float mass,
                 Vector3D velocity,
                 float radius_change_speed,
                 bint touch,
                 Vector3D touch_normal,
                 float nitro_amount,
                 int id,
                 bint is_teammate,
                 ActionStruct action):
        self.entity = Entity(position, radius, mass, velocity, radius_change_speed)
        self.touch = touch
        self.touch_normal = touch_normal
        self.nitro_amount = nitro_amount
        self.id = id
        self.is_teammate = is_teammate
        self.action = action

    def get_is_teammate(self):
        return self.is_teammate

    def get_id(self):
        return self.id

    def get_entity(self):
        return self.entity

    @staticmethod
    def from_robot(robot: Robot, action: Action, RulesStruct rules):
        return RobotEntity(Vector3D(robot.x, robot.y, robot.z),
                           robot.radius,
                           rules.ROBOT_MASS,
                           Vector3D(robot.velocity_x, robot.velocity_y, robot.velocity_z),
                           action.jump_speed,
                           robot.touch,
                           Vector3D(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z) if robot.touch else None,
                           robot.nitro_amount,
                           robot.id,
                           robot.is_teammate,
                           ActionStruct(
                               action.target_velocity_x,
                               action.target_velocity_y,
                               action.target_velocity_z,
                               action.jump_speed,
                               action.use_nitro))

cdef class BallEntity(Entity):
    cpdef Entity entity

    def __init__(self,
                 Vector3D position,
                 float radius,
                 float mass,
                 Vector3D velocity,
                 float radius_change_speed):
        self.entity = Entity(position, radius, mass, velocity, radius_change_speed)

    @staticmethod
    def from_ball(ball: Ball, RulesStruct rules):
        return BallEntity(Vector3D(ball.x, ball.y, ball.z),
                          ball.radius,
                          rules.BALL_MASS,
                          Vector3D(ball.velocity_x, ball.velocity_y, ball.velocity_z),
                          0)

    def get_entity(self):
        return self.entity

cdef class NitroEntity(Entity):
    cdef Entity entity
    cdef int nitro_amount, respawn_ticks

    def __init__(self,
                 Vector3D position,
                 float radius,
                 float mass,
                 Vector3D velocity,
                 float radius_change_speed,
                 int nitro_amount,
                 int respawn_ticks):
        self.entity = Entity(position, radius, mass, velocity, radius_change_speed)
        self.nitro_amount = nitro_amount
        self.respawn_ticks = respawn_ticks

    @staticmethod
    def from_nitro_pack(nitro_pack: NitroPack):
        return NitroEntity(position=Vector3D(nitro_pack.x, nitro_pack.y, nitro_pack.z),
                           radius=nitro_pack.radius,
                           mass=None,
                           velocity=None,
                           radius_change_speed=None,
                           nitro_amount=nitro_pack.nitro_amount,
                           respawn_ticks=nitro_pack.respawn_ticks)

cdef collide_entities__random(float x, float y):
    return min(x, y) + abs(x - y) / 2

cdef collide_entities(RulesStruct rules,
                      Entity a,
                      Entity b):
    cdef Vector3D delta_position, normal, impulse
    cdef float distance, penetration, k_a, k_b

    delta_position = b.position.sub(a.position)
    distance = delta_position.len()
    penetration = a.radius + b.radius - distance
    if penetration > 0:
        k_a = (1 / a.mass) / ((1 / a.mass) + (1 / b.mass))
        k_b = (1 / b.mass) / ((1 / a.mass) + (1 / b.mass))
        normal = delta_position.normalize()
        a.position = a.position.sub(normal.mul(penetration * k_a))
        b.position = b.position.sub(normal.mul(penetration * k_b))
        delta_velocity = normal.dot(b.velocity.sub(a.velocity)) + b.radius_change_speed - a.radius_change_speed
        if delta_velocity < 0:
            impulse = normal.mul((1 + collide_entities__random(rules.MIN_HIT_E, rules.MAX_HIT_E)) * delta_velocity)
            a.velocity = a.velocity.add(impulse.mul(k_a))
            b.velocity = a.velocity.sub(impulse.mul(k_b))

cdef Vector3D collide_with_arena(RulesStruct rules, Entity e):
    cdef Dan dan
    cdef float penetration, velocity, arena_e

    dan = dan_to_arena(rules.arena, e.position)
    penetration = e.radius - dan.distance
    if penetration > 0:
        e.position = e.position.add(dan.normal.mul(penetration))
        velocity = e.velocity.dot(dan.normal) - e.radius_change_speed
        if velocity < 0:
            if isinstance(e, RobotEntity):
                arena_e = rules.ROBOT_ARENA_E
            else:
                arena_e = rules.BALL_ARENA_E
            e.velocity = e.velocity.sub(dan.normal.mul((1 + arena_e) * velocity))
            return dan.normal
    return None

cdef move(RulesStruct rules, Entity e, float delta_time):
    e.velocity = clamp_vector(e.velocity, rules.MAX_ENTITY_SPEED)
    e.position = e.position.add(e.velocity.mul(delta_time))
    e.position.y -= rules.GRAVITY * delta_time * delta_time / 2
    e.velocity.y -= rules.GRAVITY * delta_time

cdef update(RulesStruct rules,
            float delta_time,
            robots: List[RobotEntity],
            BallEntity ball,
            nitros: List[NitroEntity],
            GameState game_state):
    cdef RobotEntity robot, robot2
    cdef NitroEntity nitro

    random.shuffle(robots[:])
    for robot0 in robots:
        robot = robot0
        target_velocity = Vector3D(robot.action.target_velocity_x, robot.action.target_velocity_y, robot.action.target_velocity_z)
        if robot.touch:
            target_velocity = target_velocity.min(rules.ROBOT_MAX_GROUND_SPEED)
            target_velocity = target_velocity.sub(robot.touch_normal.mul(robot.touch_normal.dot(target_velocity)))
            target_velocity_change = target_velocity.sub(robot.entity.velocity)
            if target_velocity_change.len() > 0:
                acceleration = rules.ROBOT_ACCELERATION * max(0, robot.touch_normal.y)
                robot.entity.velocity = robot.entity.velocity.add((target_velocity_change.normalize().mul(acceleration * delta_time)).min(target_velocity_change.len()))
        if robot.action.use_nitro:
            target_velocity_change = (target_velocity.sub(robot.entity.velocity)).min(robot.nitro_amount * rules.NITRO_POINT_VELOCITY_CHANGE)
            if target_velocity_change.len() > 0:
                acceleration = target_velocity_change.normalize().mul(rules.ROBOT_NITRO_ACCELERATION)
                velocity_change = (acceleration.mul(delta_time)).min(target_velocity_change.len())
                robot.entity.velocity = robot.entity.velocity.add(velocity_change)
                robot.nitro_amount -= velocity_change.len() / rules.NITRO_POINT_VELOCITY_CHANGE

        move(rules, robot.entity, delta_time)
        robot.entity.radius = rules.ROBOT_MIN_RADIUS + (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed / rules.ROBOT_MAX_JUMP_SPEED
        robot.entity.radius_change_speed = robot.action.jump_speed

    move(rules, ball.entity, delta_time)

    for i in range(len(robots)):
        for j in range(i):
            robot = robots[i]
            robot2 = robots[j]
            collide_entities(rules, robot.entity, robot2.entity)

    for robot0 in robots:
        robot = robot0
        collide_entities(rules, robot.entity, ball.entity)
        collision_normal = collide_with_arena(rules, robot.entity)
        if collision_normal is None:
            robot.touch = False
        else:
            robot.touch = True
            robot.touch_normal = collision_normal
    collide_with_arena(rules, ball.entity)
    if abs(ball.position.z) > rules.arena.depth / 2 + ball.radius:
        if ball.position.z > 0: # TODO: check it
            game_state.my_score += 1
        else:
            game_state.enemy_score += 1

    for robot0 in robots:
        robot = robot0
        if robot.nitro_amount == rules.MAX_NITRO_AMOUNT:
            continue
        for nitro0 in nitros:
            nitro = nitro0
            if nitro.nitro_amount == 0:
                continue
            if (robot.entity.position - nitro.entity.position).len() <= robot.entity.radius + nitro.entity.radius:
                robot.nitro_amount = rules.MAX_NITRO_AMOUNT
                nitro.nitro_amount = 0
                nitro.respawn_ticks = rules.NITRO_PACK_RESPAWN_TICKS

cdef tick(RulesStruct rules,
          robots: List[RobotEntity],
          BallEntity ball,
          nitros: List[NitroEntity],
          GameState game_state):
    cdef float delta_time = 1 / rules.TICKS_PER_SECOND
    #for _ in range(rules.MICROTICKS_PER_TICK):
    #    update(rules, delta_time / rules.MICROTICKS_PER_TICK, robots, ball, nitros, game_state)
    update(rules, delta_time, robots, ball, nitros, game_state)
    for pack in nitros:
        if pack.nitro_amount > 0:
            continue
        pack.respawn_ticks -= 1
        if pack.respawn_ticks == 0:
            pack.nitro_amount = rules.NITRO_PACK_AMOUNT

cdef class Engine:
    cdef RulesStruct rules
    cdef GameState game_state
    cpdef robots
    cpdef BallEntity ball
    cdef nitros

    def __init__(self, me: Robot, rules: Rules, game: Game):
        self.rules = RulesStruct(
            rules.max_tick_count,
            ArenaStruct(
                rules.arena.width,
                rules.arena.height,
                rules.arena.depth,
                rules.arena.bottom_radius,
                rules.arena.top_radius,
                rules.arena.corner_radius,
                rules.arena.goal_top_radius,
                rules.arena.goal_width,
                rules.arena.goal_height,
                rules.arena.goal_depth,
                rules.arena.goal_side_radius),
            rules.team_size,
            rules.seed,
            rules.ROBOT_MIN_RADIUS,
            rules.ROBOT_MAX_RADIUS,
            rules.ROBOT_MAX_JUMP_SPEED,
            rules.ROBOT_ACCELERATION,
            rules.ROBOT_NITRO_ACCELERATION,
            rules.ROBOT_MAX_GROUND_SPEED,
            rules.ROBOT_ARENA_E,
            rules.ROBOT_RADIUS,
            rules.ROBOT_MASS,
            rules.TICKS_PER_SECOND,
            rules.MICROTICKS_PER_TICK,
            rules.RESET_TICKS,
            rules.BALL_ARENA_E,
            rules.BALL_RADIUS,
            rules.BALL_MASS,
            rules.MIN_HIT_E,
            rules.MAX_HIT_E,
            rules.MAX_ENTITY_SPEED,
            rules.MAX_NITRO_AMOUNT,
            rules.START_NITRO_AMOUNT,
            rules.NITRO_POINT_VELOCITY_CHANGE,
            rules.NITRO_PACK_X,
            rules.NITRO_PACK_Y,
            rules.NITRO_PACK_Z,
            rules.NITRO_PACK_RADIUS,
            rules.NITRO_PACK_AMOUNT,
            rules.NITRO_PACK_RESPAWN_TICKS,
            rules.GRAVITY)
        self.game_state = GameState(game)
        self.robots = []
        for r in game.robots:
            action = Action()
            self.robots.append(RobotEntity.from_robot(r, action, self.rules))
        self.ball = BallEntity.from_ball(game.ball, self.rules)
        self.nitros = []
        for n in game.nitro_packs:
            self.nitros.append(NitroEntity.from_nitro_pack(n))

    def tick(self):
        start = time.time()
        tick(self.rules, self.robots, self.ball, self.nitros, self.game_state)
        print('tick in %.2f ms' % (1000 * (time.time() - start)))

    def get_robots(self):
        return self.robots

    def get_ball(self):
        return self.ball