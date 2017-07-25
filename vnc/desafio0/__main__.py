#!/usr/bin/env python

from pyguki import simulator
from . import desafio

WIDTH = 9
HEIGHT = 5
OX = 0.8
OY = 0.8
OBSTACLES = []

simulator.run(width = WIDTH, height = HEIGHT, ox = OX, oy = OY,
              obstacles = OBSTACLES, goal = (2.24, 0.5, 0.6, 0.6),
              callbacks = {
                "init":         getattr(desafio, "quando_inicia"),
                "bump_center":  getattr(desafio, "quando_bate_na_frente", None),
                "bump_left":    getattr(desafio, "quando_bate_na_esquerda", None),
                "bump_right":   getattr(desafio, "quando_bate_na_direita", None),
                "walk_done":    getattr(desafio, "andar_feito", None),
                "rotate_done":  getattr(desafio, "rodar_feito", None)
              })
