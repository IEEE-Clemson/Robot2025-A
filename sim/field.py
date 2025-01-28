from random import random
from typing import List
from pyray import *
import raylib

import pymunk

CSC_WIDTH = 0.15
GEO_RADIUS = 0.02

class SimField:
    def __init__(self, space: pymunk.Space):
        self.space: pymunk.Space = space
        self.create_field_hitbox()
        
        self.cscs: List[pymunk.Body] = []
        self.nebulite: List[pymunk.Body] = []
        self.geodynium: List[pymunk.Body] = []
        
        # CSCs
        # TODO: Verify positions
        self.add_csc(-0.5, 0.5)
        self.add_csc(0.16, -0.5)

        # Geodynium
        C = 1/39.3701
        left = (-48 + 13) * C
        right = (48 - 31.25 -6.5) * C
        top = 45/2*C
        bottom = -45/2*C

        i = 0
        while i < 6 + 8:
            x = left + random() * (right - left)
            y = bottom + random() * (top - bottom)
            self.add_am(x, y, True)
            i += 1


    def draw(self):
        # Scale from inches to meters to make this easier to write
        rl_push_matrix()
        rl_scalef(1/39.3701, 1/39.3701, 1)

        raylib.DrawRectangleV(Vector2(-48,   -24      ), (96, 48),  BLACK) # Field

        # Edges
        raylib.DrawRectangleV(Vector2(-48,   -24      ), (96, 1.5), DARKGRAY) # bottom
        raylib.DrawRectangleV(Vector2(-48,    24 - 1.5), (96, 1.5), DARKGRAY) # top
        raylib.DrawRectangleV(Vector2(-48,   -24      ), (1.5, 48), DARKGRAY) # left
        raylib.DrawRectangleV(Vector2( 48 - 1.5, -24  ), (1.5, 48), DARKGRAY) # right

        raylib.DrawRectangleV(Vector2(48 - 1.5 - 31.25 - 6.5,  24 - 1.5 - 14.5), (6.5, 14.5), DARKGRAY) # interior top
        raylib.DrawRectangleV(Vector2(48 - 1.5 - 31.25 - 6.5, -24 + 1.5), (6.5, 14.5), DARKGRAY)        # interior bottom

        # Scoring Areas
        for i in range(4):
            raylib.DrawRectangleV(Vector2(-48 + 1.5, -24 + 10 + i * 9), Vector2(12, 1), WHITE)   # horizontal lines
        raylib.DrawRectangleV(Vector2(-48 + 1.5 + 12, -24 + 1.5), Vector2(1, 48 - 2*1.5), WHITE) # vertical lines

        # Start Zone
        raylib.DrawRectangleV(Vector2(-48 + 1.5 + 24.25, -24 + 1.5), Vector2(14, 13), WHITE) # borders
        raylib.DrawRectangleV(Vector2(-48 + 1.5 + 25.25, -24 + 1.5), Vector2(12, 12), BLACK) # inside

        # Middle lines
        raylib.DrawRectangleV(Vector2(-48 + 1.5 + 23.25, -24 + 18.5 + 1.5), Vector2(69.75 - 23.25, 1), WHITE) # bottom line
        raylib.DrawRectangleV(Vector2(-48 + 1.5 + 23.25, -24 + 25.5 + 1.5), Vector2(69.75 - 23.25, 1), WHITE) # top line

        rl_pop_matrix()
        self.draw_cscs()
        self.draw_am()

    def create_field_hitbox(self):
        C = 1 / 39.3701
        static_body = self.space.static_body

        # top edge
        shape = pymunk.Segment(static_body, (48*C, (24 - 1.5 /2)*C), (-48*C, (24 - 1.5 /2)*C), 1.5 / 2*C)
        self.space.add(shape)
        shape.elasticity = 1
        shape.friction = 1

        # bottom edge
        shape = pymunk.Segment(static_body, (48*C, (-24 + 1.5 / 2)*C), (-48*C, (-24 + 1.5 / 2)*C), 1.5 / 2*C)
        self.space.add(shape)
        shape.elasticity = 1
        shape.friction = 1

        # right edge
        shape = pymunk.Segment(static_body, ((48 - 1.5/2)*C, -24*C), ((48 - 1.5/2)*C, 24*C), 1.5/2*C)
        self.space.add(shape)
        shape.elasticity = 1
        shape.friction = 1
        
        # left edge
        shape = pymunk.Segment(static_body, (-(48 - 1.5/2)*C, -24*C), (-(48 - 1.5/2)*C, 24*C), 1.5/2*C)
        self.space.add(shape)
        shape.elasticity = 1
        shape.friction = 1

        # interior top
        shape = pymunk.Poly.create_box_bb(static_body, pymunk.BB(left=(48 - 1.5 - 31.25 - 6.5)*C, bottom=(24 - 1.5 - 14.5)*C, right=(48 - 1.5 - 31.25)*C, top=(24 - 1.5)*C))
        self.space.add(shape)
        shape.elasticity = 1
        shape.friction = 1
        # interior bottom
        shape = pymunk.Poly.create_box_bb(static_body, pymunk.BB(left=(48 - 1.5 - 31.25 - 6.5)*C, bottom=(-24 + 1.5)*C, right=(48 - 1.5 - 31.25)*C, top=(-24 + 1.5 + 14.5)*C))
        self.space.add(shape)
        shape.elasticity = 1
        shape.friction = 1


    def add_csc(self, x: float, y: float):
        csc_body = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
        csc_body.position = x, y
        csc_body.mass = 0.5
        csc_shape = pymunk.Poly.create_box(csc_body, (CSC_WIDTH, CSC_WIDTH))
        csc_shape.mass = csc_body.mass
        csc_shape.body = csc_body
        csc_shape.friction = 1
        csc_body.moment = csc_shape.moment
        self.space.add(csc_shape, csc_body)
        self.cscs.append(csc_body)

    def draw_cscs(self):
        for csc in self.cscs:
            rl_push_matrix()
            rl_translatef(csc.position[0], csc.position[1], 0)
            rl_rotatef(csc.angle, 0, 0, 1)
            draw_rectangle_v(Vector2(-CSC_WIDTH / 2, -CSC_WIDTH / 2), Vector2(CSC_WIDTH, CSC_WIDTH), LIGHTGRAY)
            rl_pop_matrix()

    def add_am(self, x: float, y: float, is_geodynium: bool):
        am_body = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
        am_body.position = x, y
        am_body.mass = 0.05
        am_shape = pymunk.Circle(am_body, GEO_RADIUS)
        am_shape.mass = am_body.mass
        am_shape.body = am_body
        am_body.moment = am_shape.moment
        am_shape.friction = 1
        self.space.add(am_shape, am_body)
        if is_geodynium:
            self.geodynium.append(am_body)
        else:
            self.nebulite.append(am_body)

    def draw_am(self):
        for am in self.geodynium + self.nebulite:
            draw_circle_v(Vector2(*am.position), GEO_RADIUS, PURPLE)