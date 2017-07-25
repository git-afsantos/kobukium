#!/usr/bin/env python

import random
from pyguki import simulator
from . import desafio

WIDTH = 9
HEIGHT = 5
OX = random.uniform(0.2, 1.0)
OY = random.uniform(0.7, 1.4)
OBSTACLES = [(2, 4), (3, 4), (4, 4), (0, 7), (1, 7), (2, 7)]

simulator.run(width = WIDTH, height = HEIGHT, ox = OX, oy = OY, obstacles = OBSTACLES,
              goal = (7*0.32, 3*0.32, 0.64, 0.64),
              callbacks = {
                "init":         getattr(desafio, "quando_inicia"),
                "bump_center":  getattr(desafio, "quando_bate_na_frente", None),
                "bump_left":    getattr(desafio, "quando_bate_na_esquerda", None),
                "bump_right":   getattr(desafio, "quando_bate_na_direita", None),
                "walk_done":    getattr(desafio, "andar_feito", None),
                "rotate_done":  getattr(desafio, "rodar_feito", None)
              })
