from pyray import *
import raylib

import pymunk


def draw_field():
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

def create_field_hitbox(space: pymunk.Space):
    C = 1 / 39.3701
    static_body = space.static_body

    # top edge
    shape = pymunk.Segment(static_body, (48*C, (24 - 1.5 /2)*C), (-48*C, (24 - 1.5 /2)*C), 1.5 / 2*C)
    space.add(shape)
    shape.elasticity = 1
    shape.friction = 1

    # bottom edge
    shape = pymunk.Segment(static_body, (48*C, (-24 + 1.5 / 2)*C), (-48*C, (-24 + 1.5 / 2)*C), 1.5 / 2*C)
    space.add(shape)
    shape.elasticity = 1
    shape.friction = 1

    # right edge
    shape = pymunk.Segment(static_body, ((48 - 1.5/2)*C, -24*C), ((48 - 1.5/2)*C, 24*C), 1.5/2*C)
    space.add(shape)
    shape.elasticity = 1
    shape.friction = 1
    
    # left edge
    shape = pymunk.Segment(static_body, (-(48 - 1.5/2)*C, -24*C), (-(48 - 1.5/2)*C, 24*C), 1.5/2*C)
    space.add(shape)
    shape.elasticity = 1
    shape.friction = 1

    # interior top
    shape = pymunk.Poly.create_box_bb(static_body, pymunk.BB(left=(48 - 1.5 - 31.25 - 6.5)*C, bottom=(24 - 1.5 - 14.5)*C, right=(48 - 1.5 - 31.25)*C, top=(24 - 1.5)*C))
    space.add(shape)
    shape.elasticity = 1
    shape.friction = 1
    # interior bottom
    shape = pymunk.Poly.create_box_bb(static_body, pymunk.BB(left=(48 - 1.5 - 31.25 - 6.5)*C, bottom=(-24 + 1.5)*C, right=(48 - 1.5 - 31.25)*C, top=(-24 + 1.5 + 14.5)*C))
    space.add(shape)
    shape.elasticity = 1
    shape.friction = 1


    # shape = pymunk.Segment(static_body, (-1.22, -0.61), (1.22, -0.61), 0.0125)
    # space.add(shape)
    # shape.elasticity = 1
    # shape.friction = 1
