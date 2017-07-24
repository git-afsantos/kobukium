#!/usr/bin/env python

from math import pi
from pyguki import simulator
import solucao2

WIDTH = 9
HEIGHT = 4
OX = 0.2
OY = 0.32
OA = pi/4
OBSTACLES = [(3, 0), (3, 1), (3, 2), (2, 1),
             (0, 3), (0, 4), (0, 5), (1, 4),
             (3, 7), (3, 8), (2, 8)]

simulator.run(width = WIDTH, height = HEIGHT, ox = OX, oy = OY, oa = OA,
              obstacles = OBSTACLES,
              goal = (1.92, 0, 0.96, 1.28),
              callbacks = {
                "init":         getattr(solucao2, "quando_inicia"),
                "bump_center":  getattr(solucao2, "quando_bate_na_frente", None),
                "bump_left":    getattr(solucao2, "quando_bate_na_esquerda", None),
                "bump_right":   getattr(solucao2, "quando_bate_na_direita", None),
                "walk_done":    getattr(solucao2, "andar_feito", None),
                "rotate_done":  getattr(solucao2, "rodar_feito", None)
              })
