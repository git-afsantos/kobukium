#!/usr/bin/env python

from pyguki import simulator
import solucao

WIDTH = 9
HEIGHT = 5
OX = 1.0
OY = 1.0
OBSTACLES = [(0, 4)]

simulator.run(width = WIDTH, height = HEIGHT, ox = OX, oy = OY, obstacles = OBSTACLES,
              callbacks = {
                "init":         getattr(solucao, "quando_inicia"),
                "bump_center":  getattr(solucao, "quando_bate_na_frente"),
                "bump_left":    getattr(solucao, "quando_bate_na_esquerda"),
                "bump_right":   getattr(solucao, "quando_bate_na_direita"),
                "walk_done":    getattr(solucao, "andar_feito"),
                "rotate_done":  getattr(solucao, "rodar_feito")
              })
