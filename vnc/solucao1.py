from math import pi, sin, cos, sqrt

# robot.andar(m)
# robot.rodar(rad)
# robot.terminar()
# robot.executar_depois("comando", valor)


def quando_inicia(robot):
    robot.rodar(pi/4)

def quando_bate_na_frente(robot):
    pass

def quando_bate_na_esquerda(robot):
    pass

def quando_bate_na_direita(robot):
    pass

def andar_feito(robot):
    robot.rodar(-pi/2)

def rodar_feito(robot):
    robot.andar(sqrt(0.64**2 + 0.64**2))
