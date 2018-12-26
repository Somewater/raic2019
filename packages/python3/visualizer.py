import pyglet
from pyglet.window import key, mouse
import ratcave as rc
import os
from engine import Engine

delta = -6.5

class Visualizer:
    def __init__(self, root):
        self.root = root
        self.started = False
        self.keys = key.KeyStateHandler()
        self.mouse = mouse.RIGHT
        self.robot_by_id = dict()

    def build_arena(self, engine: Engine):
        reader = rc.WavefrontReader(os.path.join(self.root, 'arena.obj'))
        mesh_name = list(reader.bodies.keys())[0]
        meshes = []
        self.arena = reader.get_mesh(mesh_name)
        self.arena.position.xyz = 0, 0, -1
        self.arena.scale.xyz = 0.01, 0.01, 0.01
        self.arena.rotation.xyz = 53, 43, 0
        meshes.append(self.arena)

        obj_filename = rc.resources.obj_primitives
        obj_reader = rc.WavefrontReader(obj_filename)
        self.ball = obj_reader.get_mesh("Sphere")
        self.ball.scale.xyz = 2,2,2
        self.ball.textures.append(rc.Texture.from_image(os.path.join(self.root, 'localrunner/assets/ball/texture.png')))
        self.arena.add_child(self.ball)
        meshes.append(self.ball)

        for robot in engine.robot_entities:
            robot_mesh = obj_reader.get_mesh("Sphere")
            if robot.is_teammate:
                robot_mesh.textures.append(rc.Texture.from_image(os.path.join(self.root, 'localrunner/assets/robot/green/texture.png')))
            else:
                robot_mesh.textures.append(rc.Texture.from_image(os.path.join(self.root, 'localrunner/assets/robot/orange/texture.png')))
            self.robot_by_id[robot.id] = robot_mesh
            self.arena.add_child(robot_mesh)
            meshes.append(robot_mesh)

        self.scene = rc.Scene(meshes=meshes)
        self.scene.light.position = 0, 0, 0

    def move_camera(self, dt):
        if self.keys[key.LEFT]:
            self.scene.camera.position.x += 0.01 * dt
        elif self.keys[key.RIGHT]:
            self.scene.camera.position.x -= 0.01 * dt

        if self.keys[key.UP]:
            self.scene.camera.position.y += 0.01 * dt
        elif self.keys[key.DOWN]:
            self.scene.camera.position.y -= 0.01 * dt

        if self.keys[key.PAGEUP]:
            self.scene.camera.position.z += 0.01 * dt
        elif self.keys[key.PAGEDOWN]:
            self.scene.camera.position.z -= 0.01 * dt

        if self.keys[key.W]:
            self.scene.light.position.x += 0.01 * dt
        elif self.keys[key.S]:
            self.scene.light.position.x -= 0.01 * dt

        if self.keys[key.A]:
            self.scene.light.position.y += 0.1 * dt
        elif self.keys[key.D]:
            self.scene.light.position.y -= 0.1 * dt

        global delta
        if self.keys[key.Q]:
            self.scene.light.position.z += 0.1 * dt
        elif self.keys[key.E]:
            self.scene.light.position.z -= 0.1 * dt


        self.scene.camera.look_at(self.arena.position.x, self.arena.position.y, self.arena.position.z)
        self.scene.light.look_at(self.arena.position.x, self.arena.position.y, self.arena.position.z)

    def update_positions(self, engine: Engine):
        global delta
        ball_pos =  engine.ball_entity.position
        self.ball.position.xyz = ball_pos.get_x(), ball_pos.get_y() + delta, ball_pos.get_z()
        for robot in engine.robot_entities:
            robot_pos = robot.position
            self.robot_by_id[robot.id].position.xyz = robot_pos.get_x(), robot_pos.get_y() + delta, robot_pos.get_z()

    def start(self, engine: Engine):
        if not self.started:
            self.started = True
            self.window = window = pyglet.window.Window(width=1200, height=900)
            self.window.push_handlers(self.keys)

            @window.event
            def on_draw():
                with rc.default_shader:
                    self.scene.draw()

            @window.event
            def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
                scale = 0.1 if modifiers & key.MOD_CTRL else 1.0
                self.arena.rotation.x = max(0, self.arena.rotation.x + dy * scale)
                self.arena.rotation.y += -dx * scale

            @window.event
            def on_mouse_scroll(x, y, scroll_x, scroll_y):
                self.scene.camera.position.z += scroll_y * 0.01

            self.build_arena(engine)
        else:
            with rc.default_shader:
                self.scene.draw()
            self.window.flip()

            self.update_positions(engine)
            pyglet.clock.schedule(self.move_camera)
            # pyglet.app.run()
            pyglet.clock.tick()
            # self.move_camera(0)
            for window in pyglet.app.windows:
                window.dispatch_events()
