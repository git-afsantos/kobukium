
#Copyright (c) 2017 Andre Santos
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

import time
from math import pi, cos, sin, sqrt, isnan, degrees
import pygame as pg
import sys

TWO_PI = 2 * pi
PI_2 = pi / 2
PI_6 = pi / 6
TILE_SIZE = 32
ARENA_WIDTH = 6
ARENA_HEIGHT = 9
LINEAR  = 0.25  # m/s
ANGULAR = pi/4  # rad/s

def angle_distance(alpha, beta):
    d = alpha - beta
    s = 1 if (d >= 0.0 and d <= pi) or (d <= -pi and d >= -TWO_PI) else -1
    d = abs(d) % TWO_PI
    r = TWO_PI - d if d > pi else d
    return r * s


class Spritesheet(object):
    def __init__(self, filename):
        try:
            self.sheet = pg.image.load(filename).convert_alpha()
        except pg.error, message:
            print "Unable to load spritesheet image: " + filename
            raise SystemExit, message

    def image_at(self, rectangle, colorkey = None):
        rect = pg.Rect(rectangle)
        image = pg.Surface(rect.size, pg.SRCALPHA, 32).convert_alpha()
        image.blit(self.sheet, (0, 0), rect)
        if not colorkey is None:
            if colorkey is -1:
                colorkey = image.get_at((0,0))
            image.set_colorkey(colorkey, pg.RLEACCEL)
        return image

    def images_at(self, rectangles, colorkey = None):
        return [self.image_at(rect, colorkey) for rect in rectangles]

    def load_strip(self, rect, image_count, margin = 0, colorkey = None):
        tups = [(rect[0] + (rect[2] + margin) * x, rect[1], rect[2], rect[3])
                for x in range(image_count)]
        return self.images_at(tups, colorkey)



class ImageSequence(object):
    def __init__(self, spritesheet, rectangle, image_count, delays,
                 margin = 0, colorkey = None):
        if isinstance(delays, (int, long, float)):
            delays = [delays for n in xrange(image_count)]
        elif len(delays) < image_count:
            last = delays[-1]
            delays.extend([last for n in xrange(image_count - len(delays))])
        # for i in xrange(1, len(delays)):
            # delays[i] = delays[i-1] + delays[i]
        self.images = spritesheet.load_strip(rectangle, image_count,
                                        margin = margin, colorkey = colorkey)
        self.delays = delays
        self.elapsed = 0
        self._i = 0

    @property
    def sprite(self):
        return self.images[self._i]

    def reset(self):
        self.elapsed = 0
        self._i = 0

    def update(self, dt):
        self.elapsed += dt
        while self.elapsed >= self.delays[self._i]:
            self.elapsed -= self.delays[self._i]
            self._i = (self._i + 1) % len(self.delays)



class MultiPoseSprite(object):
    def __init__(self):
        self.sprites = {}
        self.current = None
        self._sprite = None

    @property
    def sprite(self):
        return self._sprite.sprite

    def add_sprite(self, name, sprite):
        self.sprites[name] = sprite

    def set_sprite(self, name):
        assert name in self.sprites
        if name != self.current:
            self.current = name
            self._sprite = self.sprites[name]
            self._sprite.reset()

    def update(self, dt):
        self._sprite.update(dt)



class Odometry(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.a = 0.0
        self.vx = 0.0
        self.wz = 0.0
    # -- Saving previous values
        self._x = 0.0
        self._y = 0.0
        self._a = 0.0
        self._vx = 0.0
        self._wz = 0.0

    @property
    def xcm(self):
        return self.x * 100.0

    @property
    def ycm(self):
        return self.y * 100.0

    @property
    def ix(self):
        return int(self.x)

    @property
    def iy(self):
        return int(self.y)

    @property
    def moved(self):
        return sqrt((self.x - self._x)**2 + (self.y - self._y)**2)

    @property
    def has_moved(self):
        return self._x != self.x or self._y != self.y

    @property
    def has_moved_left(self):
        return self.x < self._x

    @property
    def has_moved_right(self):
        return self.x > self._x

    @property
    def has_moved_up(self):
        return self.y < self._y

    @property
    def has_moved_down(self):
        return self.y > self._y

    @property
    def rotated(self):
        return angle_distance(self.a, self._a)

    @property
    def has_rotated(self):
        return self._a != self.a

    def reset(self, x = 0.0, y = 0.0, a = 0.0):
        self._x = self.x
        self._y = self.y
        self._a = self.a
        self.x = x
        self.y = y
        self.a = a

    def update(self, dr, da, dt):
        self._x = self.x
        self._y = self.y
        self._a = self.a
        self._vx = self.vx
        self._wz = self.wz
        self.x += dr * cos(self.a)
        self.y += dr * sin(-self.a)
        self.a += da
        self.vx = dr / dt
        self.wz = da / dt
        while self.a > pi:
            self.a -= TWO_PI
        while self.a < -pi:
            self.a += TWO_PI

    def snap_x(self, x):
        self.x = x
        self.y = self._y
        self.a = self._a
        self.vx = 0.0
        self.wz = 0.0

    def snap_y(self, y):
        self.x = self._x
        self.y = y
        self.a = self._a
        self.vx = 0.0
        self.wz = 0.0

    def snap_xy(self, x, y):
        self.x = x
        self.y = y
        self.a = self._a
        self.vx = 0.0
        self.wz = 0.0



class DifferentialDrive(object):
    def __init__(self, max_linear = 0.5, max_angular = 2.0,
                 wheel_separation = 0.23, wheel_diameter = 0.070):
        self.cmd_left           = 0.0
        self.cmd_right          = 0.0
        self.max_linear         = max_linear        # m/s
        self.max_angular        = max_angular       # rad/s
        self.wheel_separation   = wheel_separation
        self.wheel_radius       = wheel_diameter / 2

    def set_velocity_commands(self, linear, angular):
        linear = min(linear, self.max_linear)
        linear = max(linear, -self.max_linear)
        angular = min(angular, self.max_angular)
        angular = max(angular, -self.max_angular)
        self.cmd_left = linear - angular * self.wheel_separation / 2
        self.cmd_right = linear + angular * self.wheel_separation / 2

    def get_odom_velocity(self, dt):
        d1 = dt * self.wheel_radius * (self.cmd_left / self.wheel_radius)
        d2 = dt * self.wheel_radius * (self.cmd_right / self.wheel_radius)
        if isnan(d1):
            d1 = 0.0
        if isnan(d2):
            d2 = 0.0
        dr = (d1 + d2) / 2
        da = (d2 - d1) / self.wheel_separation
        return (dr, da)



class Bumper(object):
    def __init__(self):
        self.center     = False
        self.left       = False
        self.right      = False
    # -- Previous values
        self._center    = False
        self._left      = False
        self._right     = False

    @property
    def active(self):
        return self.center or self.left or self.right

    @property
    def changed(self):
        return (self.center != self._center or self.left != self._left
                or self.right != self._right)

    def set_center(self, active):
        self._center = self.center
        self.center = active

    def set_left(self, active):
        self._left = self.left
        self.left = active

    def set_right(self, active):
        self._right = self.right
        self.right = active

    def release(self):
        self._center = self.center
        self._left  = self.left
        self._right = self.right
        self.center = False
        self.left   = False
        self.right  = False

    def from_angle(self, a, b = None):
        while a > pi:
            a -= TWO_PI
        while a < -pi:
            a += TWO_PI
        if not b is None:
            while b > pi:
                b -= TWO_PI
            while b < -pi:
                b += TWO_PI
        if a < -PI_2 or a > PI_2:
            if b is None:
                return self.release()
            return self.from_angle(b)
        if not b is None and (b < -PI_2 or b > PI_2):
            b = None
        c = False
        l = False
        r = False
        if a < -PI_6:
            r = True
        elif a > PI_6:
            l = True
        else:
            c = True
        if not b is None:
            if b < -PI_6:
                r = True
            elif b > PI_6:
                l = True
            else:
                c = True
        self.set_center(c)
        self.set_left(l)
        self.set_right(r)

    def collision_str(self):
        if self.center:
            if self.left:
                if self.right:
                    return "collision-clr"
                return "collision-cl"
            if self.right:
                return "collision-cr"
            return "collision-c"
        if self.left:
            if self.right:
                return "collision-lr"
            return "collision-l"
        if self.right:
            return "collision-r"
        return "normal"



class Robot(pg.sprite.Sprite):
    def __init__(self, game, odom = None, drive = None, radius = 16,
                 sprites = None):
        pg.sprite.Sprite.__init__(self)
        self.game   = game
        self.odom   = odom or Odometry()
        self.drive  = drive or DifferentialDrive()
        self.bumper = Bumper()
        self.r      = radius
        self.d      = radius * 2
        self.sprites = sprites or MultiPoseSprite()
        self.image  = None
        self.set_image("normal")

    # def kill():
    #   remove the Sprite from all Groups

    # def alive():
    #   does the sprite belong to any groups

    def update(self, dt):
        dr, da = self.drive.get_odom_velocity(dt)
        self.odom.update(dr, da, dt)
        self._detect_collision()

    def draw(self, screen):
        if self.odom.has_rotated:
            angle = degrees(self.odom.a)
            original = self.sprites.sprite
            self.image = pg.transform.rotate(original, angle)
            self.rect = self.image.get_rect(center = original.get_rect().center)
        self.rect.centerx = int(self.odom.xcm)
        self.rect.centery = int(self.odom.ycm)
        screen.blit(self.image, (self.rect.x, self.rect.y))

    @property
    def row(self):
        return self.odom.iy // TILE_SIZE

    @property
    def col(self):
        return self.odom.ix // TILE_SIZE

    def set_image(self, name):
        if name == self.sprites.current:
            return
        self.sprites.set_sprite(name)
        image = self.sprites.sprite
        if image is None:
            self.image = pg.Surface([self.d, self.d])
            self.image.fill((0, 0, 0))
            self.rect = self.image.get_rect()
        else:
            angle = degrees(self.odom.a)
            self.image = pg.transform.rotate(image, angle)
            self.rect = self.image.get_rect(center = image.get_rect().center)
        self.rect.centerx = int(self.odom.xcm)
        self.rect.centery = int(self.odom.ycm)

    def update_image(self):
        image = self.sprites.sprite
        angle = degrees(self.odom.a)
        self.image = pg.transform.rotate(image, angle)
        self.rect = self.image.get_rect(center = image.get_rect().center)
        self.rect.centerx = int(self.odom.xcm)
        self.rect.centery = int(self.odom.ycm)


    def _detect_collision(self):
        x1 = self.odom.xcm - self.r
        x2 = self.odom.xcm + self.r
        y1 = self.odom.ycm - self.r
        y2 = self.odom.ycm + self.r
        arena = self.game.arena
        cu = arena.is_wall_xy(self.odom.xcm, y1)
        cd = arena.is_wall_xy(self.odom.xcm, y2)
        cl = arena.is_wall_xy(x1, self.odom.ycm)
        cr = arena.is_wall_xy(x2, self.odom.ycm)
        if cu:
            if cl:
                self.odom.snap_xy((arena.x_right(x1) + self.r) / 100.0,
                                  (arena.y_below(y1) + self.r) / 100.0)
                self.bumper.from_angle(PI_2 - self.odom.a,
                                       b = pi - self.odom.a)
            elif cr:
                self.odom.snap_xy((arena.x_left(x2) - self.r) / 100.0,
                                  (arena.y_below(y1) + self.r) / 100.0)
                self.bumper.from_angle(PI_2 - self.odom.a,
                                       b = -self.odom.a)
            else:
                if self.odom.has_moved_up:
                    self.odom.snap_y((arena.y_below(y1) + self.r) / 100.0)
                self.bumper.from_angle(PI_2 - self.odom.a)
        elif cd:
            if cl:
                self.odom.snap_xy((arena.x_right(x1) + self.r) / 100.0,
                                  (arena.y_above(y2) - self.r) / 100.0)
                self.bumper.from_angle(-PI_2 - self.odom.a,
                                       b = pi - self.odom.a)
            elif cr:
                self.odom.snap_xy((arena.x_left(x2) - self.r) / 100.0,
                                  (arena.y_above(y2) - self.r) / 100.0)
                self.bumper.from_angle(-PI_2 - self.odom.a,
                                       b = -self.odom.a)
            else:
                if self.odom.has_moved_down:
                    self.odom.snap_y((arena.y_above(y2) - self.r) / 100.0)
                self.bumper.from_angle(-PI_2 - self.odom.a)
        else:
            if cl:
                if self.odom.has_moved_left:
                    self.odom.snap_x((arena.x_right(x1) + self.r) / 100.0)
                self.bumper.from_angle(pi - self.odom.a)
            elif cr:
                if self.odom.has_moved_right:
                    self.odom.snap_x((arena.x_left(x2) - self.r) / 100.0)
                self.bumper.from_angle(-self.odom.a)
            else:
                self.bumper.release()
        if cu or cd or cl or cr:
            self.drive.set_velocity_commands(0.0, 0.0)



class Keyop(object):
    def __init__(self, robot):
        self.robot = robot
        self.left = False
        self.right = False
        self.up = False
        self.down = False

    def update(self):
        vx = 0.0
        wz = 0.0
        if self.up:
            vx = 0.25
        elif self.down:
            vx = -0.125
        if self.left:
            wz = 1.0
        elif self.right:
            wz = -1.0
        self.robot.drive.set_velocity_commands(vx, wz)

    def on_key_up(self, event):
        if event.key == pg.K_LEFT or event.key == ord('a'):
            self.left = False
            self.update()
        if event.key == pg.K_RIGHT or event.key == ord('d'):
            self.right = False
            self.update()
        if event.key == pg.K_UP or event.key == ord('w'):
            self.up = False
            self.update()
        if event.key == pg.K_DOWN or event.key == ord('s'):
            self.down = False
            self.update()

    def on_key_down(self, event):
        if event.key == pg.K_LEFT or event.key == ord('a'):
            self.right = False
            self.left = True
            self.update()
        if event.key == pg.K_RIGHT or event.key == ord('d'):
            self.left = False
            self.right = True
            self.update()
        if event.key == pg.K_UP or event.key == ord('w'):
            self.down = False
            self.up = True
            self.update()
        if event.key == pg.K_DOWN or event.key == ord('s'):
            self.up = False
            self.down = True
            self.update()


class DefaultInput(object):
    def on_key_up(self, event):
        pass

    def on_key_down(self, event):
        pass



class UserController(object):
    def __init__(self, robot, callbacks, goal):
        self.robot          = robot
        self.goal           = goal
        self.enabled        = True
        self.to_walk        = 0.0
        self.to_rotate      = 0.0
        self.vx             = 0.0
        self.wz             = 0.0
        self.contador       = 0
        self.commands       = []
        self.init           = callbacks.get("init")         or self.skip
        self.bump_center    = callbacks.get("bump_center")  or self.skip
        self.bump_left      = callbacks.get("bump_left")    or self.skip
        self.bump_right     = callbacks.get("bump_right")   or self.skip
        self.walk_done      = callbacks.get("walk_done")    or self.skip
        self.rotate_done    = callbacks.get("rotate_done")  or self.skip
        self._center        = False
        self._left          = False
        self._right         = False
        self._time          = time.time()

    def update(self, dt):
        if self.enabled:
            if self.to_walk > 0.0:
                self.to_walk -= self.robot.odom.moved
                if self.to_walk <= 0.0:
                    self.to_walk = 0.0
                    self.vx = 0.0
                    self.wz = 0.0
                    self.walk_done(self)
                self.robot.drive.set_velocity_commands(self.vx, self.wz)
            if self.to_rotate > 0.0:
                self.to_rotate -= abs(self.robot.odom.rotated)
                if self.to_rotate <= 0.0:
                    self.to_rotate = 0.0
                    self.vx = 0.0
                    self.wz = 0.0
                    self.rotate_done(self)
                self.robot.drive.set_velocity_commands(self.vx, self.wz)
        if self.robot.bumper.center and not self._center:
            self._center = True
            self.bump_center(self)
        elif self.robot.bumper.left and not self._left:
            self._left = True
            self.bump_left(self)
        elif self.robot.bumper.right and not self._right:
            self._right = True
            self.bump_right(self)
        elif not self.robot.bumper.active:
            self._center = False
            self._left   = False
            self._right  = False

    def skip(self, robot):
        if self.commands:
            cmd, val = self.commands.pop(0)
            cmd(val)
        else:
            self.terminar()

    def andar(self, meters):
        self.vx = LINEAR
        self.wz = 0.0
        self.to_walk = meters
        self.to_rotate = 0.0

    def rodar(self, radians):
        self.vx = 0.0
        self.to_walk = 0.0
        if radians > 0:
            self.wz = ANGULAR
            self.to_rotate = radians
        elif radians < 0:
            self.wz = -ANGULAR
            self.to_rotate = -radians

    def executar_depois(self, cmd, value):
        self.commands.append((getattr(self, cmd), value))

    def cancelar_comando(self):
        if self.to_walk > 0.0 or self.to_rotate > 0.0:
            self.to_walk = 0.0
            self.to_rotate = 0.0
            self.vx = 0.0
            self.wz = 0.0
            if self.commands:
                cmd, val = self.commands.pop(0)
                cmd(val)

    def conta(self):
        self.contador += 1

    def desconta(self):
        self.contador -= 1

    def terminar(self):
        self.vx = 0.0
        self.wz = 0.0
        self.to_walk = 0.0
        self.to_rotate = 0.0
        if not self.goal:
            print "[TERMINADO]"
        else:
            x = self.robot.odom.x
            y = self.robot.odom.y
            x1 = self.goal[0]
            x2 = self.goal[0] + self.goal[2]
            y1 = self.goal[1]
            y2 = self.goal[1] + self.goal[3]
            if x >= x1 and x < x2 and y >= y1 and y < y2:
                t = time.time() - self._time
                print "[RESULTADO]: Sucesso! {0:.2f} segundos.".format(t)
            else:
                print "[RESULTADO]: Fracasso!"


class SafetyController(object):
    def __init__(self, robot):
        self.robot = robot
        self.timer = 0
        self.vx = 0.0
        self.wz = 0.0

    @property
    def active(self):
        return self.timer > 0

    def update(self, dt):
        if self.timer > 0:
            self.timer -= dt
            self.robot.drive.set_velocity_commands(self.vx, self.wz)
            if self.timer <= 0:
                self.vx = 0.0
                self.wz = 0.0
                self.robot.drive.set_velocity_commands(0.0, 0.0)
                self.robot.set_image(self.robot.bumper.collision_str())
        elif self.robot.bumper.active:
            self.timer = 1.0
            self.robot.set_image(self.robot.bumper.collision_str())
            if self.robot.bumper.center:
                self.vx = -0.1
                self.wz = 0.0
            elif self.robot.bumper.left:
                self.vx = -0.1
                self.wz = -0.4
            elif self.robot.bumper.right:
                self.vx = -0.1
                self.wz = 0.4



class Arena(object):
    def __init__(self, rows, columns, obstacles):
        self.rows = rows
        self.columns = columns
        self.obstacles = obstacles

    def is_wall_xy(self, x, y):
        return self.is_wall(int(y) // TILE_SIZE, int(x) // TILE_SIZE)

    def is_wall(self, row, col):
        if row < 0 or row >= self.rows:
            return True
        if col < 0 or col >= self.columns:
            return True
        if (row, col) in self.obstacles:
            return True
        return False

    def y_below(self, y):
        return ((int(y) // TILE_SIZE) + 1) * TILE_SIZE - 1.0

    def y_above(self, y):
        return (int(y) // TILE_SIZE) * TILE_SIZE

    def x_right(self, x):
        return ((int(x) // TILE_SIZE) + 1) * TILE_SIZE - 1.0

    def x_left(self, x):
        return (int(x) // TILE_SIZE) * TILE_SIZE






class State(object):
    def __init__(self):
        self.done = False
        self.next = None
        self.quit = False
        self.previous = None

    # def cleanup(self):
    # def startup(self):
    # def get_event(self, event):
    # def update(self, dt):
    # def draw(self, screen):


class Game(State):
    def __init__(self, width, height, ox, oy, oa, img_path, callbacks,
                 obstacles, goal):
        State.__init__(self)
        self.next = "game"
        self.goal = (int(goal[0] * 100), int(goal[1] * 100),
                     int(goal[2] * 100), int(goal[3] * 100)) if goal else None
        self.arena = Arena(height, width, obstacles)
        self.robot = Robot(self, radius = 18, sprites = load_images(img_path))
        if not callbacks:
            self.input = Keyop(self.robot)
        else:
            self.input = DefaultInput()
        self.safety = SafetyController(self.robot)
        self.user = UserController(self.robot, callbacks or {}, goal)
        self.robot.odom.reset(x = ox, y = oy, a = oa)
        self.robot.odom.reset(x = ox, y = oy, a = oa)
        # Kobuki diameter: 35.15cm

    def cleanup(self):
        print("cleaning up Game state stuff")

    def startup(self):
        print("starting Game state stuff")
        self.user.init(self.user)
        self.robot.update_image()

    def get_event(self, event):
        if event.type == pg.KEYDOWN:
            self.input.on_key_down(event)
        elif event.type == pg.KEYUP:
            if event.key == pg.K_ESCAPE:
                pg.quit()
                sys.exit()
            else:
                self.input.on_key_up(event)
        elif event.type == pg.MOUSEBUTTONDOWN:
            self.done = True

    def update(self, dt):
        # NOTE: safety update here so that it overrides keyop
        self.safety.update(dt)
        self.user.enabled = not self.safety.active
        self.user.update(dt)
        self.robot.update(dt)

    def draw(self, screen):
        screen.fill((224,224,224))
        if self.goal:
            screen.fill((255, 192, 192), rect = self.goal)
        for i in xrange(self.arena.rows):
            for j in xrange(self.arena.columns):
                if self.arena.is_wall(i, j):
                    screen.fill((128, 128, 128), rect = (32 * j, 32 * i, 32, 32))
        w = self.arena.columns * TILE_SIZE
        h = self.arena.rows * TILE_SIZE
        for i in xrange(1, self.arena.rows):
            pg.draw.line(screen, (0,0,0), (0, i * TILE_SIZE), (w, i * TILE_SIZE))
        for i in xrange(1, self.arena.columns):
            pg.draw.line(screen, (0,0,0), (i * TILE_SIZE, 0), (i * TILE_SIZE, h))
        self.robot.draw(screen)


class Control(object):
    def __init__(self, **settings):
        self.__dict__.update(settings)
        self.done = False
        self.screen = pg.display.set_mode(self.size)
        self.clock = pg.time.Clock()

    def setup_states(self, state_dict, start_state):
        self.state_dict = state_dict
        self.state_name = start_state
        self.state = self.state_dict[self.state_name]

    def flip_state(self):
        self.state.done = False
        previous, self.state_name = self.state_name, self.state.next
        self.state.cleanup()
        self.state = self.state_dict[self.state_name]
        self.state.startup()
        self.state.previous = previous

    def update(self, dt):
        if self.state.quit:
            self.done = True
        elif self.state.done:
            self.flip_state()
        self.state.update(dt)
        self.state.draw(self.screen)
        pg.display.flip()

    def event_loop(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.done = True
            self.state.get_event(event)

    def main_game_loop(self):
        self.state.startup()
        while not self.done:
            delta_time = self.clock.tick(self.fps)/1000.0
            self.event_loop()
            self.update(delta_time)
            pg.display.update()




def load_images(img_path):
    spritesheet = Spritesheet(img_path + "spritesheet.png")
    sprite = MultiPoseSprite()
    seq = ImageSequence(spritesheet, (0, 0, 36, 36), 1, 1)
    sprite.add_sprite("normal", seq)
    seq = ImageSequence(spritesheet, (36, 0, 36, 36), 1, 1)
    sprite.add_sprite("collision-c", seq)
    seq = ImageSequence(spritesheet, (36*2, 0, 36, 36), 1, 1)
    sprite.add_sprite("collision-l", seq)
    seq = ImageSequence(spritesheet, (36*3, 0, 36, 36), 1, 1)
    sprite.add_sprite("collision-r", seq)
    seq = ImageSequence(spritesheet, (36*4, 0, 36, 36), 1, 1)
    sprite.add_sprite("collision-cl", seq)
    seq = ImageSequence(spritesheet, (36*5, 0, 36, 36), 1, 1)
    sprite.add_sprite("collision-cr", seq)
    seq = ImageSequence(spritesheet, (36*6, 0, 36, 36), 1, 1)
    sprite.add_sprite("collision-clr", seq)
    seq = ImageSequence(spritesheet, (36*7, 0, 36, 36), 1, 1)
    sprite.add_sprite("collision-lr", seq)
    return sprite



def run(width = ARENA_WIDTH, height = ARENA_HEIGHT, ox = 0.8, oy = 0.8, oa = 0.0,
        obstacles = None, img_path = "pyguki/images/", callbacks = None,
        goal = None):
    obstacles = obstacles or []
    settings = {
        "size": (width * TILE_SIZE, height * TILE_SIZE),
        "fps" : 60
    }

    app = Control(**settings)
    state_dict = {
        "game": Game(width, height, ox, oy, oa, img_path, callbacks, obstacles,
                     goal)
    }
    app.setup_states(state_dict, "game")
    app.main_game_loop()
    pg.quit()
    sys.exit()
    


if __name__ == "__main__":
    run(img_path = "images/", obstacles = [(5,3)])
