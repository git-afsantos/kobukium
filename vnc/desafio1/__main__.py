#!/usr/bin/env python

from math import pi
from pyguki import simulator
from . import desafio

WIDTH = 7
HEIGHT = 7
OX = 0.32 * 3.5
OY = 0.32 * 5.5
OA = pi/2
OBSTACLES = [(3, 3)]

simulator.run(width = WIDTH, height = HEIGHT, ox = OX, oy = OY, oa = OA,
              obstacles = OBSTACLES,
              goal = (OX - 0.16, OY - 0.16, 0.32, 0.32),
              callbacks = {
                "init":         getattr(desafio, "quando_inicia"),
                "bump_center":  getattr(desafio, "quando_bate_na_frente", None),
                "bump_left":    getattr(desafio, "quando_bate_na_esquerda", None),
                "bump_right":   getattr(desafio, "quando_bate_na_direita", None),
                "walk_done":    getattr(desafio, "andar_feito", None),
                "rotate_done":  getattr(desafio, "rodar_feito", None)
              })
