# distutils: language = c++
# distutils: sources = /Users/pnaydenov/lab/raic2018/packages/c++17/engine.cpp /Users/pnaydenov/lab/raic2018/packages/c++17/predefined.cpp /Users/pnaydenov/lab/raic2018/packages/c++17/vector3d.cpp

from libcpp cimport bool
from libcpp.vector cimport vector

cdef extern from "../c++17/vector3d.h":
    cdef cppclass Vector3D:
        double x;
        double y;
        double z;

cdef extern from "../c++17/engine.h":
    cdef cppclass GameState:
        GameState()
        int my_score
        int enemy_score

cdef extern from "../c++17/engine.h":
    cdef cppclass Entity:
        Vector3D position;
        Vector3D velocity;

cdef extern from "../c++17/engine.h":
    cdef cppclass RobotEntity:
        Vector3D position;
        Vector3D velocity;
        int id;
        bint is_teammate;

cdef extern from "../c++17/engine.h":
    cdef cppclass BallEntity:
        Vector3D position;
        Vector3D velocity;

cdef extern from "../c++17/engine.h":
    cdef cppclass Engine:
        Engine(const int me, const Rules& rules, const Game& game)
        void simulate()
        BallEntity ball;
        vector[RobotEntity] robots;

cdef extern from "../c++17/model/Robot.h":
    cdef cppclass Robot:
        Robot()
        int id;
        int player_id;
        bool is_teammate;
        double x;
        double y;
        double z;
        double velocity_x;
        double velocity_y;
        double velocity_z;
        double radius;
        double nitro_amount;
        bool touch;
        double touch_normal_x;
        double touch_normal_y;
        double touch_normal_z;

cdef extern from "../c++17/model/Ball.h":
    cdef cppclass Ball:
        Ball();
        double x;
        double y;
        double z;
        double velocity_x;
        double velocity_y;
        double velocity_z;
        double radius;

cdef extern from "../c++17/model/Player.h":
    cdef cppclass Player:
        Player();
        int id;
        bool me;
        bool strategy_crashed;
        int score;

cdef extern from "../c++17/model/NitroPack.h":
    cdef cppclass NitroPack:
        NitroPack();
        int id;
        double x;
        double y;
        double z;
        double radius;
        bool alive;
        int respawn_ticks;

cdef extern from "../c++17/model/Arena.h":
    cdef cppclass Arena:
        Arena();
        double width;
        double height;
        double depth;
        double bottom_radius;
        double top_radius;
        double corner_radius;
        double goal_top_radius;
        double goal_width;
        double goal_height;
        double goal_depth;
        double goal_side_radius;

cdef extern from "../c++17/model/Rules.h":
    cdef cppclass Rules:
        Rules();
        int max_tick_count;
        Arena arena;
        int team_size;
        long long seed;
        double ROBOT_MIN_RADIUS;
        double ROBOT_MAX_RADIUS;
        double ROBOT_MAX_JUMP_SPEED;
        double ROBOT_ACCELERATION;
        double ROBOT_NITRO_ACCELERATION;
        double ROBOT_MAX_GROUND_SPEED;
        double ROBOT_ARENA_E;
        double ROBOT_RADIUS;
        double ROBOT_MASS;
        int TICKS_PER_SECOND;
        int MICROTICKS_PER_TICK;
        int RESET_TICKS;
        double BALL_ARENA_E;
        double BALL_RADIUS;
        double BALL_MASS;
        double MIN_HIT_E;
        double MAX_HIT_E;
        double MAX_ENTITY_SPEED;
        double MAX_NITRO_AMOUNT;
        double START_NITRO_AMOUNT;
        double NITRO_POINT_VELOCITY_CHANGE;
        double NITRO_PACK_X;
        double NITRO_PACK_Y;
        double NITRO_PACK_Z;
        double NITRO_PACK_RADIUS;
        double NITRO_PACK_AMOUNT;
        int NITRO_PACK_RESPAWN_TICKS;
        double GRAVITY;

cdef extern from "../c++17/model/Game.h":
    cdef cppclass Game:
        Game()
        int current_tick;
        vector[Player] players;
        vector[Robot] robots;
        vector[NitroPack] nitro_packs;
        Ball ball;

cdef build_vec(Vector3D vec):
    return _Vec(vec.x, vec.y, vec.z)

class _Vec:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def get_x(self): return self.x
    def get_y(self): return self.y
    def get_z(self): return self.z

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return '(%f, %f, %f)' % (self.x, self.y, self.z)

class EntityHolder:
    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity

    def get_entity(self):
        return self

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity

class _Robot(EntityHolder):
    def __init__(self, position, velocity, is_teammate, id):
        EntityHolder.__init__(self, position, velocity)
        self.is_teammate = is_teammate
        self.id = id

    def get_is_teammate(self):
        return self.is_teammate

    def get_id(self):
        return self.id

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return 'R_%d(position=%s, velocity=%s)' % (self.id, self.position, self.velocity)

class _Ball(EntityHolder):
    pass

cdef class PyEngine:
    cdef Engine* engine;
    def __cinit__(self, int id, rules, game):
        cdef Robot robot_c
        cdef Player player_c
        cdef NitroPack nitro_c
        cdef Rules rules_c
        cdef Game game_c = Game()
        for robot in game.robots:
            robot_c = Robot()
            robot_c.id = robot.id
            robot_c.player_id = robot.player_id
            robot_c.is_teammate = robot.is_teammate
            robot_c.x = robot.x
            robot_c.y = robot.y
            robot_c.z = robot.z
            robot_c.velocity_x = robot.velocity_x
            robot_c.velocity_y = robot.velocity_y
            robot_c.velocity_z = robot.velocity_z
            robot_c.radius = robot.radius
            robot_c.nitro_amount = robot.nitro_amount
            robot_c.touch = robot.touch
            if robot_c.touch:
                robot_c.touch_normal_x = robot.touch_normal_x
                robot_c.touch_normal_y = robot.touch_normal_y
                robot_c.touch_normal_z = robot.touch_normal_z
            else:
                robot_c.touch_normal_x = 0
                robot_c.touch_normal_y = 0
                robot_c.touch_normal_z = 0
            game_c.robots.push_back(robot_c)

        for player in game.players:
            player_c = Player()
            player_c.id = player.id
            player_c.me = player.me
            player_c.strategy_crashed = player.strategy_crashed
            player_c.score = player.score
            game_c.players.push_back(player_c)

        for nitro in game.nitro_packs:
            nitro_c = NitroPack()
            nitro_c.id = nitro.id
            nitro_c.x = nitro.x
            nitro_c.y = nitro.y
            nitro_c.z = nitro.z
            nitro_c.radius = nitro.radius
            nitro_c.alive = nitro.alive
            nitro_c.respawn_ticks = nitro.respawn_ticks
            game_c.nitro_packs.push_back(nitro_c)

        game_c.ball.x = game.ball.x
        game_c.ball.y = game.ball.y
        game_c.ball.z = game.ball.z
        game_c.ball.velocity_x = game.ball.velocity_x
        game_c.ball.velocity_y = game.ball.velocity_y
        game_c.ball.velocity_z = game.ball.velocity_z
        game_c.ball.radius = game.ball.radius

        game_c.current_tick = game.current_tick

        rules_c.max_tick_count = rules.max_tick_count

        rules_c.arena.width = rules.arena.width
        rules_c.arena.height = rules.arena.height
        rules_c.arena.depth = rules.arena.depth
        rules_c.arena.bottom_radius = rules.arena.bottom_radius
        rules_c.arena.top_radius = rules.arena.top_radius
        rules_c.arena.corner_radius = rules.arena.corner_radius
        rules_c.arena.goal_top_radius = rules.arena.goal_top_radius
        rules_c.arena.goal_width = rules.arena.goal_width
        rules_c.arena.goal_height = rules.arena.goal_height
        rules_c.arena.goal_depth = rules.arena.goal_depth
        rules_c.arena.goal_side_radius = rules.arena.goal_side_radius

        rules_c.team_size = rules.team_size
        rules_c.seed = rules.seed
        rules_c.ROBOT_MIN_RADIUS = rules.ROBOT_MIN_RADIUS
        rules_c.ROBOT_MAX_RADIUS = rules.ROBOT_MAX_RADIUS
        rules_c.ROBOT_MAX_JUMP_SPEED = rules.ROBOT_MAX_JUMP_SPEED
        rules_c.ROBOT_ACCELERATION = rules.ROBOT_ACCELERATION
        rules_c.ROBOT_NITRO_ACCELERATION = rules.ROBOT_NITRO_ACCELERATION
        rules_c.ROBOT_MAX_GROUND_SPEED = rules.ROBOT_MAX_GROUND_SPEED
        rules_c.ROBOT_ARENA_E = rules.ROBOT_ARENA_E
        rules_c.ROBOT_RADIUS = rules.ROBOT_RADIUS
        rules_c.ROBOT_MASS = rules.ROBOT_MASS
        rules_c.TICKS_PER_SECOND = rules.TICKS_PER_SECOND
        rules_c.MICROTICKS_PER_TICK = rules.MICROTICKS_PER_TICK
        rules_c.RESET_TICKS = rules.RESET_TICKS
        rules_c.BALL_ARENA_E = rules.BALL_ARENA_E
        rules_c.BALL_RADIUS = rules.BALL_RADIUS
        rules_c.BALL_MASS = rules.BALL_MASS
        rules_c.MIN_HIT_E = rules.MIN_HIT_E
        rules_c.MAX_HIT_E = rules.MAX_HIT_E
        rules_c.MAX_ENTITY_SPEED = rules.MAX_ENTITY_SPEED
        rules_c.MAX_NITRO_AMOUNT = rules.MAX_NITRO_AMOUNT
        rules_c.START_NITRO_AMOUNT = rules.START_NITRO_AMOUNT
        rules_c.NITRO_POINT_VELOCITY_CHANGE = rules.NITRO_POINT_VELOCITY_CHANGE
        rules_c.NITRO_PACK_X = rules.NITRO_PACK_X
        rules_c.NITRO_PACK_Y = rules.NITRO_PACK_Y
        rules_c.NITRO_PACK_Z = rules.NITRO_PACK_Z
        rules_c.NITRO_PACK_RADIUS = rules.NITRO_PACK_RADIUS
        rules_c.NITRO_PACK_AMOUNT = rules.NITRO_PACK_AMOUNT
        rules_c.NITRO_PACK_RESPAWN_TICKS = rules.NITRO_PACK_RESPAWN_TICKS
        rules_c.GRAVITY = rules.GRAVITY

        self.engine = new Engine(id, rules_c, game_c)

    def simulate(self):
        self.engine.simulate()

    def __dealloc__(self):
        del self.engine

    def get_robots(self):
        result = []
        for i in range(self.engine.robots.size()):
            result.append(_Robot(build_vec(self.engine.robots.at(i).position),
                                 build_vec(self.engine.robots.at(i).velocity),
                                 self.engine.robots.at(i).is_teammate,
                                 self.engine.robots.at(i).id))
        return result

    def get_ball(self):
        return _Ball(build_vec(self.engine.ball.position),
                     build_vec(self.engine.ball.velocity))
