from math import pi, sin, cos, sqrt

# robot.andar(m)
# robot.rodar(rad)

metros = 1.28 - 0.32


def quando_inicia(robot):
    robot.andar(metros)

def quando_bate_na_frente(robot):
    pass

def quando_bate_na_esquerda(robot):
    pass

def quando_bate_na_direita(robot):
    pass

def andar_feito(robot):
    robot.rodar(-pi/2)

def rodar_feito(robot):
    robot.andar(metros)
