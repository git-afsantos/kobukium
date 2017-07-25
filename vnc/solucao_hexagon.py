from math import pi, sin, cos, sqrt

# robot.andar(m)
# robot.rodar(rad)
# robot.executar_depois("comando", valor)
# robot.cancelar_comando()
# robot.contador
# robot.conta()
# robot.desconta()

def quando_inicia(robot):
    metros = sqrt(0.32**2 + 0.64**2)
    robot.rodar(pi/2)
    robot.executar_depois("andar", 0.32)
    robot.executar_depois("rodar", -pi/3)
    robot.executar_depois("andar", metros)
    robot.executar_depois("rodar", -pi/3)
    robot.executar_depois("andar", metros)
    robot.executar_depois("rodar", -pi/3)
    robot.executar_depois("andar", 0.64)
    robot.executar_depois("rodar", -pi/3)
    robot.executar_depois("andar", metros)
    robot.executar_depois("rodar", -pi/3)
    robot.executar_depois("andar", metros)
    robot.executar_depois("rodar", -pi/3)
    robot.executar_depois("andar", 0.32)

def quando_bate_na_frente(robot):
    robot.terminar()

def quando_bate_na_esquerda(robot):
    robot.terminar()

def quando_bate_na_direita(robot):
    robot.terminar()
