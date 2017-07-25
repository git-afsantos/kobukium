#!/usr/bin/env python

import random
from pyguki import simulator
from . import desafio

WIDTH = 7
HEIGHT = 7
OX = random.uniform(0.25, 0.8)
OY = random.uniform(1.28, 1.6)
OBSTACLES = [(2, 2), (2, 3), (2, 4), (3, 4), (4, 4)]

simulator.run(width = WIDTH, height = HEIGHT, ox = OX, oy = OY,
              obstacles = OBSTACLES, goal = (5*0.32, 5*0.32, 0.64, 0.64),
              callbacks = {
                "init":         getattr(desafio, "quando_inicia"),
                "bump_center":  getattr(desafio, "quando_bate_na_frente", None),
                "bump_left":    getattr(desafio, "quando_bate_na_esquerda", None),
                "bump_right":   getattr(desafio, "quando_bate_na_direita", None),
                "walk_done":    getattr(desafio, "andar_feito", None),
                "rotate_done":  getattr(desafio, "rodar_feito", None)
              })
