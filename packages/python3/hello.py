import json
from engine_cpp import PyEngine
from model import Robot, Game, Rules
from visualizer import Visualizer
import os
import time

GAME = '{"current_tick":0,"players":[{"id":1,"me":true,"strategy_crashed":false,"score":0},{"id":2,"me":false,"strategy_crashed":false,"score":0}],"robots":[{"id":1,"player_id":1,"is_teammate":true,"x":4.465929725872228,"y":1.0,"z":-19.495011456359055,"velocity_x":0.0,"velocity_y":0.0,"velocity_z":0.0,"radius":1.0,"nitro_amount":0.0,"touch":true,"touch_normal_x":0.0,"touch_normal_y":1.0,"touch_normal_z":0.0},{"id":2,"player_id":1,"is_teammate":true,"x":-14.650210305339492,"y":1.0,"z":-13.615114322300954,"velocity_x":0.0,"velocity_y":0.0,"velocity_z":0.0,"radius":1.0,"nitro_amount":0.0,"touch":true,"touch_normal_x":0.0,"touch_normal_y":1.0,"touch_normal_z":0.0},{"id":3,"player_id":2,"is_teammate":false,"x":-4.465929725872228,"y":1.0,"z":19.495011456359055,"velocity_x":0.0,"velocity_y":0.0,"velocity_z":0.0,"radius":1.0,"nitro_amount":0.0,"touch":true,"touch_normal_x":0.0,"touch_normal_y":1.0,"touch_normal_z":0.0},{"id":4,"player_id":2,"is_teammate":false,"x":14.650210305339492,"y":1.0,"z":13.615114322300954,"velocity_x":0.0,"velocity_y":0.0,"velocity_z":0.0,"radius":1.0,"nitro_amount":0.0,"touch":true,"touch_normal_x":0.0,"touch_normal_y":1.0,"touch_normal_z":0.0}],"nitro_packs":[],"ball":{"x":0.0,"y":2.4690013713158567,"z":0.0,"velocity_x":0.0,"velocity_y":0.0,"velocity_z":0.0,"radius":2.0}}\n'
RULES = '{"seed":2,"max_tick_count":18000,"arena":{"width":60.0,"height":20.0,"depth":80.0,"bottom_radius":3.0,"top_radius":7.0,"corner_radius":13.0,"goal_top_radius":3.0,"goal_width":30.0,"goal_height":10.0,"goal_depth":10.0,"goal_side_radius":1.0},"team_size":2,"ROBOT_MIN_RADIUS":1.0,"ROBOT_MAX_RADIUS":1.05,"ROBOT_MAX_JUMP_SPEED":15.0,"ROBOT_ACCELERATION":100.0,"ROBOT_NITRO_ACCELERATION":30.0,"ROBOT_MAX_GROUND_SPEED":30.0,"ROBOT_ARENA_E":0.0,"ROBOT_RADIUS":1.0,"ROBOT_MASS":2.0,"TICKS_PER_SECOND":60,"MICROTICKS_PER_TICK":100,"RESET_TICKS":120,"BALL_ARENA_E":0.7,"BALL_RADIUS":2.0,"BALL_MASS":1.0,"MIN_HIT_E":0.4,"MAX_HIT_E":0.5,"MAX_ENTITY_SPEED":100.0,"MAX_NITRO_AMOUNT":100.0,"START_NITRO_AMOUNT":50.0,"NITRO_POINT_VELOCITY_CHANGE":0.6,"NITRO_PACK_X":20.0,"NITRO_PACK_Y":1.0,"NITRO_PACK_Z":30.0,"NITRO_PACK_RADIUS":0.5,"NITRO_PACK_AMOUNT":100.0,"NITRO_PACK_RESPAWN_TICKS":600,"GRAVITY":30.0}\n'

if __name__ == '__main__':
    script_root = os.path.dirname(os.path.realpath(__file__))
    project_root = os.path.realpath(os.path.join(script_root, '..', '..'))
    rules = Rules(json.loads(RULES))
    game = Game(json.loads(GAME))
    me = None
    for r in game.robots:
        if r.is_teammate:
            me = r
            break

    visualizer = Visualizer(project_root)
    engine = PyEngine(me.id, game, rules)
    visualizer.start(engine)
    for i in range(1000000):
        engine.simulate()
        visualizer.start(engine)
        time.sleep(0.01)
    print(engine)