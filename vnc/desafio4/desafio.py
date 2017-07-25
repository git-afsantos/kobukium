from math import pi, sin, cos, sqrt

# robot.andar(m)
# robot.rodar(rad)
# robot.executar_depois("comando", valor)
# robot.cancelar_comando()
# robot.contador
# robot.conta()
# robot.desconta()


def quando_inicia(robot):
    robot.contador = 5
    robot.andar(10)

def quando_bate_na_frente(robot):
    robot.desconta()
    if robot.contador > 0:
        robot.rodar(pi/2)
    else:
        robot.terminar()

def quando_bate_na_esquerda(robot):
    pass

def quando_bate_na_direita(robot):
    pass

def andar_feito(robot):
    pass

def rodar_feito(robot):
    robot.andar(10)
