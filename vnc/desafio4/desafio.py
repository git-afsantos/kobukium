from math import pi, sin, cos, sqrt

# Cada tela mede 0.32 metros de lado.
# O robot tem 0.18 metros de raio.
# O robot tem 0.18 metros de raio.
# Para este desafio, o robot começa algures entre (0.25, 1.28) e (0.8, 1.6)
# orientado para 0 radianos.
# O objetivo é o retângulo de (1.6, 1.6) até (2.24, 2.24).
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
