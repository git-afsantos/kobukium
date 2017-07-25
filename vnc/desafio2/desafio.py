from math import pi, sin, cos, sqrt

# Cada tela mede 0.32 metros de lado.
# O robot tem 0.18 metros de raio.
# Para este desafio, o robot começa na posição (0.2, 0.32)
# orientado para pi/4 radianos.
# O objectivo é o retângulo de (1.92, 0) até (0.96, 1.28).
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
    robot.rodar(-pi/4)
    robot.executar_depois("andar", 0.44)
    robot.executar_depois("rodar", -pi/4)
    robot.executar_depois("andar", sqrt(0.64**2 + 0.64**2))
    robot.executar_depois("rodar", pi/4)
    robot.executar_depois("andar", 0.64)
    robot.executar_depois("rodar", pi/4)
    robot.executar_depois("andar", sqrt(0.32**2 + 0.32**2))

def quando_bate_na_frente(robot):
    robot.terminar()

def quando_bate_na_esquerda(robot):
    robot.terminar()

def quando_bate_na_direita(robot):
    robot.terminar()