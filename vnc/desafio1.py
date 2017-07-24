#!/usr/bin/env python

from pyguki import simulator
import solucao1

WIDTH = 7
HEIGHT = 5
OX = 0.64
OY = 0.32
OBSTACLES = []

simulator.run(width = WIDTH, height = HEIGHT, ox = OX, oy = OY, obstacles = OBSTACLES,
              callbacks = {
                "init":         getattr(solucao1, "quando_inicia"),
                "bump_center":  getattr(solucao1, "quando_bate_na_frente"),
                "bump_left":    getattr(solucao1, "quando_bate_na_esquerda"),
                "bump_right":   getattr(solucao1, "quando_bate_na_direita"),
                "walk_done":    getattr(solucao1, "andar_feito"),
                "rotate_done":  getattr(solucao1, "rodar_feito")
              })
