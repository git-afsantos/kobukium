from math import pi, sin, cos, sqrt

# robot.andar(m)
# robot.rodar(rad)
# robot.executar_depois("comando", valor)
# robot.cancelar_comando()
# robot.contador
# robot.conta()
# robot.desconta()

def quando_inicia(robot):
    robot.contador = 7
    robot.andar(2)

def quando_bate_na_frente(robot):
    robot.terminar()

def quando_bate_na_esquerda(robot):
    robot.desconta()
    if robot.contador == 0:
        robot.terminar()
    else:
        robot.rodar(-3*pi/8)

def quando_bate_na_direita(robot):
    robot.desconta()
    if robot.contador == 0:
        robot.terminar()
    else:
        robot.rodar(3*pi/8)

def rodar_feito(robot):
    robot.andar(2)
