from math import pi, sin, cos, sqrt

# robot.andar(m)
# robot.rodar(rad)

def quando_inicia(robot):
    print "iniciou"
    robot.andar(0.25)

def quando_bate_na_frente(robot):
    print "bateu na frente"

def quando_bate_na_esquerda(robot):
    print "bateu na esquerda"

def quando_bate_na_direita(robot):
    print "bateu na direita"

def andar_feito(robot):
    print "andou tudo"
    robot.rodar(pi/2)

def rodar_feito(robot):
    print "rodou tudo"
    robot.andar(0.25)
