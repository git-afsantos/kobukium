from math import pi, sin, cos, sqrt

# Cada tela mede 0.32 metros de lado.
# O robot tem 0.18 metros de raio.
# Para este desafio, o robot começa algures entre (0.2, 0.7) e (1, 1.4)
# orientado para 0 radianos.
# O objetivo é o retângulo de (2.24, 0.96) até (2.88, 1.6).
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

etapas_andar = [10, 10, 10, 0.5]
etapas_rodar = [pi/2, -pi/2, -pi/2, pi/2]

def quando_inicia(robot):
    robot.andar(10)
    robot.executar_depois("rodar", pi/2)
    robot.executar_depois("andar", 10)
    robot.executar_depois("rodar", -pi/2)
    robot.executar_depois("andar", 10)
    robot.executar_depois("rodar", -pi/2)
    robot.executar_depois("andar", 10)
    robot.executar_depois("rodar", pi/2)
    robot.executar_depois("andar", 0.5)
