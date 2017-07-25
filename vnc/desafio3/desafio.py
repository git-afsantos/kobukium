from math import pi, sin, cos, sqrt

# robot.andar(m)
# robot.rodar(rad)
# robot.executar_depois("comando", valor)
# robot.cancelar_comando()
# robot.contador
# robot.conta()
# robot.desconta()

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
