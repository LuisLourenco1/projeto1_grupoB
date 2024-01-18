#!/usr/bin/env python3

import numpy as np
import heapq
import cv2
import rospy
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import tf

# Constante para a velocidade máxima

# Constante para a velocidade angular máxima



class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Custo do caminho
        self.h = 0  # Heurística
        self.f = 0  # Custo total

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

def astar(map, start, end):
    print("Calculando o caminho...")
    # Cria nó inicial e final
    start_node = Node(start, None)
    end_node = Node(end, None)

    # Inicializa as listas aberta e fechada
    open_list = []
    closed_list = []

    # Adiciona o nó inicial
    heapq.heappush(open_list, start_node)

    # Loop até encontrar o fim
    while len(open_list) > 0:
        # Pega o nó atual
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Encontrou o caminho
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Retorna o caminho invertido

        # Gera filhos
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacentes
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Verifica a validade da posição
            if not map.is_valid_position(node_position):
                continue

            # Cria novo nó
            new_node = Node(node_position, current_node)

            # Nó já está na lista fechada
            if new_node in closed_list:
                continue

            # Cria os valores f, g e h
            new_node.g = current_node.g + 1
            new_node.h = ((new_node.position[0] - end_node.position[0]) ** 2) + ((new_node.position[1] - end_node.position[1]) ** 2)
            new_node.f = new_node.g + new_node.h

            # Nó já está na lista aberta
            if any(child for child in open_list if new_node == child and child.g < new_node.g):
                continue

            # Adiciona o nó à lista aberta
            heapq.heappush(open_list, new_node)

    return None

class Map:
    def __init__(self, map_file):
        self.map = self.load_map(map_file)

    def load_map(self, map_file):
        # Carregar o arquivo .pgm e converter para uma matriz numpy
        map_image = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
        if map_image is None:
            raise ValueError("Não foi possível carregar o arquivo do mapa.")

        # Aqui, assumimos que áreas brancas (255) são livres e áreas escuras são obstáculos
        threshold = 127
        map_array = (map_image > threshold).astype(np.int64)
        return map_array

    def is_valid_position(self, position):
        # Verificar se a posição está dentro dos limites do mapa
        if position[0] < 0 or position[0] >= self.map.shape[0] or \
           position[1] < 0 or position[1] >= self.map.shape[1]:
            return False

        # Verificar se a posição está livre de obstáculos (1 representa área livre)
        return self.map[position[0], position[1]] == 1
    
class RoboAStar:
    def __init__(self, map_file, start, goal):
        self.map = Map(map_file)
        self.start = start
        self.goal = goal
        self.current_position = start
        self.current_orientation = 0
        self.path = astar(self.map, self.start, self.goal)
        self.obstacle_detected = False

        # Inicializa o nó ROS
        rospy.init_node('robo_astar', anonymous=True)

        # Subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.rviz_path_pub = rospy.Publisher('/rviz_path', Path, queue_size=10)
        self.map_pub = rospy.Publisher('/map_topic', OccupancyGrid, queue_size=10)

    def map_callback(self, map_file):
        self.map_data = np.array(map_file.data).reshape((map_file.info.width, map_file.info.height))
        self.map_resolution = map_file.info.resolution
        self.map_width = map_file.info.width
        self.map_height = map_file.info.height

        # Publica o mapa ao receber os dados
        self.publish_map()

    def publish_map(self):
        if self.map_data is not None:
            map_msg = OccupancyGrid()
            map_msg.header = Header()
            map_msg.header.stamp = rospy.Time.now()
            map_msg.header.frame_id = 'map'
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = self.map_width
            map_msg.info.height = self.map_height

            # Converte a matriz do mapa para uma lista
            map_msg.data = self.map_data.flatten().tolist()

            self.map_pub.publish(map_msg)

    def publish_path(self, path):
        if path:
            path_msg = Path()
            path_msg.header = Header()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = 'map'
            path_msg.poses = path

            self.path_pub.publish(path_msg)

            # Publica a rota no rviz
            self.rviz_path_pub.publish(path_msg)

    def odom_callback(self, data):
        self.current_position = (data.pose.pose.position.x, data.pose.pose.position.y)

        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )

        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.current_orientation = euler[2]  # yaw é o terceiro elemento

    def scan_callback(self, data):
        # Verifica se há obstáculos próximos e recalcula o caminho, se necessário
        for distance in data.ranges:
            if distance < 0.00005:  # Define um limiar de distância
                self.obstacle_detected = True
                break

    def move_robot(self):
        if not self.path:
            # Pare o robô se não houver caminho
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return

        # Implemente o controle de movimento aqui. Por exemplo, simples movimento linear até o próximo ponto.
        next_point = self.path[0]
        twist = Twist()
        twist.linear.x = min(1, self.calculate_linear_speed(next_point))
        twist.angular.z = self.calculate_angular_speed(next_point)
        self.cmd_vel_pub.publish(twist)

    def calculate_linear_speed(self, next_point):
        # Calcular a distância euclidiana até o próximo ponto
        dx = next_point[0] - self.current_position[0]
        dy = next_point[1] - self.current_position[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5

        # Velocidade proporcional à distância
        speed = min(1, distance)

        return speed

    def calculate_angular_speed(self, next_point):
        # Calcular o ângulo necessário para virar em direção ao próximo ponto
        angle_to_target = np.arctan2(next_point[1] - self.current_position[1], next_point[0] - self.current_position[0])
        
        # Calcular a diferença de ângulo
        angle_diff = angle_to_target - self.current_orientation  # 'self.current_orientation' deve ser atualizado em 'odom_callback'

        # Velocidade proporcional à diferença de ângulo
        angular_speed = min(1, abs(angle_diff)) * np.sign(angle_diff)

        return angular_speed

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        print(self.path)
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # Recalcular o caminho se um obstáculo for detectado
                self.path = astar(self.map, self.current_position, self.goal)
                self.obstacle_detected = False

            self.move_robot()

            # Verifique se o robô chegou ao ponto desejado]
            if self.path is not None:
                if self.reached_point(self.path[0]):
                    # Remova o ponto alcançado do caminho
                    self.path.pop(0)

            rate.sleep()

    def reached_point(self, point, threshold=0.1):
        # Calcular a distância até o ponto
        dx = point[0] - self.current_position[0]
        dy = point[1] - self.current_position[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5

        # Verificar se a distância é menor que o limiar
        return distance < threshold

if __name__ == '__main__':
    try:
        robo = RoboAStar('/home/luis/mapa.pgm', (0, 0), (300, 300))
        robo.run()
    except rospy.ROSInterruptException:
        pass


'''
# Exemplo de uso
map = Map('/home/luis/mapa.pgm')
print(map.is_valid_position((5, 5)))

# Exemplo de uso
start = (0, 0)
end = (5, 5)
path = astar(map, start, end)
print(path)
'''
