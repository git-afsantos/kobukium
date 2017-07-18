from math import pi, sin, cos, sqrt

# robot.andar(m)
# robot.rodar(rad)

etapas_andar = [10, 10, 10, 10, 0.5]
etapas_rodar = [pi/2, -pi/2, -pi/2, pi/2]

def quando_inicia(robot):
    print "iniciou"
    robot.andar(etapas_andar.pop(0))

def quando_bate_na_frente(robot):
    print "bateu na frente"
    robot.rodar(etapas_rodar.pop(0))

def quando_bate_na_esquerda(robot):
    print "bateu na esquerda"

def quando_bate_na_direita(robot):
    print "bateu na direita"

def andar_feito(robot):
    print "andou tudo"

def rodar_feito(robot):
    print "rodou tudo"
    robot.andar(etapas_andar.pop(0))
