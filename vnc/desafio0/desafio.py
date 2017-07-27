# -*- coding=utf-8 -*-
from math import pi, sin, cos, sqrt

# Cada tela mede 0.32 metros de lado.
# O robot tem 0.18 metros de raio.
# Para este desafio, o robot começa na posição (0.8, 0.8)
# orientado para 0 radianos.
# O objetivo é o retângulo de (2.24, 0.5) até (2.84, 1.1).
# O ponto (0, 0) é o canto superior esquerdo do mapa.
# O eixo dos X cresce para a direita, o dos Y para baixo.
# Radianos positivos rodam para a esquerda, negativos para a direita.

# robot.andar(m)    - andar em frente {m} metros
# robot.rodar(rad)  - rodar sem sair do sítio {rad} radianos
# robot.executar_depois("comando", valor)
#                   - registar um comando ("andar", "rodar")
#                     para executar depois, por ordem
# robot.cancelar_comando()
#                   - cancelar o comando atual e passar ao seguinte
# robot.contador    - número inteiro, começa em zero
# robot.conta()     - adicionar 1 ao contador
# robot.desconta()  - subtrair 1 ao contador
# robot.terminar()  - terminar a atividade do robot e
#                     verificar se chegou ao objetivo


def quando_inicia(robot):
    robot.andar(1.7)

def quando_bate_na_frente(robot):
    pass

def quando_bate_na_esquerda(robot):
    pass

def quando_bate_na_direita(robot):
    pass

def andar_feito(robot):
    robot.rodar(pi)

def rodar_feito(robot):
    robot.terminar()
