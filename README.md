# Projeto1 - GrupoB

- Augusto Luchesi Matos, BCC, 740871
- Lucas Abbiati Pereira, BCC, 801572
- Luís Fernando do Carmo Lourenço, BCC, 800210

 ## Desenvolvimento 
 Desenvolvemos o projeto em ROS, utilizando o robô do DC e o mapa pré-definido "EnvDC_2ndfloor.world", com o ponto de partida em frente ao LARIS.
 Primeiramente, foi feito o mapeamento da área através do sendor LIDAR presente no robô e foi feito um planejamento de trajetória baseado no algoritmo A*.
 
 ## Funcionamento do Algoritmo A*:

1. **Inicialização**: O algoritmo começa com um nó inicial e um nó final, ambos definidos no grafo. Também mantém uma lista de nós a serem avaliados.

2. **Cálculo das Heurísticas**: Para cada nó, o algoritmo calcula dois valores:
   - **g(n)**: O custo real do caminho do nó inicial até o nó atual.
   - **h(n)**: A heurística (estimativa) do custo do caminho do nó atual até o nó final. Esta heurística deve ser admissível, ou seja, nunca superestimar o custo real para alcançar o destino. Uma heurística comum é a distância euclidiana entre os nós no caso de um problema de busca em um espaço contínuo.

3. **Custo Total**: O custo total para alcançar um nó é dado por **f(n) = g(n) + h(n)**.

4. **Exploração dos Nós**: O algoritmo começa a explorar os nós na ordem do menor custo total estimado (f(n)). Ele seleciona o nó com o menor f(n) na lista de nós a serem avaliados.

5. **Atualização dos Custos**: Para os nós vizinhos do nó selecionado, o algoritmo atualiza os custos g(n) e f(n) se um novo caminho mais curto for encontrado. Ele mantém um registro dos nós já visitados para evitar ciclos.

6. **Critérios de Parada**: O algoritmo continua a explorar e atualizar os custos até que o nó de destino seja alcançado ou não haja mais nós para explorar na lista.

## Problemas para rodar
O algoritmo está com algum detalhe que impede seu perfeito funcionamento: ele lê o mapa, gera o caminho de forma correta, porém o robô não segue exatamente esse caminho. Acreditamos que seja algum problema entre o mapa mostrado no Gazebo e o mapa efetivamente utilizado no RVIZ. Aparentemente, não é um problema complexo de se resolver, mas não conseguimos resolvê-lo antes do prazo final.

## Project

This package contains a toolkit designed for the DC Autonomous Mobile Robot. Developed as part of the Department of Computer - Autonomous Mobile Robot (DCAMR) Project, this toolkit provides essential functionalities for the DC robot's autonomous capabilities.

 
| Operational System          	|  Ubuntu 20.04        	|
| ---------------------------- | ------------------------ |
| ROS                        	| ![image](https://user-images.githubusercontent.com/74054598/149457205-fd48db89-0658-4511-af36-bcd8662562da.png)|
| Gazebo   	              	| Gazebo multi-robot simulator - version 11.13.0 	|

**Features**
   - Host robot DC
   - RPLidar
   - Camera


## Required packages in apt-get:

```bash
sudo apt-get install ros-noetic-amcl ros-noetic-costmap-converter ros-noetic-depthimage-to-laserscan ros-noetic-dynamic-reconfigure ros-noetic-ddynamic-reconfigure ros-noetic-ddynamic-reconfigure-dbgsym ros-noetic-ddynamic-reconfigure-python ros-noetic-geometry2 ros-noetic-hector-slam ros-noetic-hector-gazebo-plugins ros-noetic-move-base ros-noetic-move-base-flex ros-noetic-navigation ros-noetic-openslam-gmapping ros-noetic-rplidar-ros ros-noetic-slam-gmapping ros-noetic-spatio-temporal-voxel-layer ros-noetic-teb-local-planner ros-noetic-teleop-twist-keyboard ros-noetic-teleop-twist-joy ros-noetic-urg-node ros-noetic-rtabmap ros-noetic-rtabmap-ros ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server ros-noetic-octovis ros-noetic-imu-filter-madgwick ros-noetic-robot-localization ros-noetic-robot-pose-ekf ros-noetic-pointcloud-to-laserscan ros-noetic-rosbridge-server ros-noetic-map-server ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins ros-noetic-ompl ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-global-planner ros-noetic-costmap-2d ros-noetic-robot-self-filter ros-noetic-ros-numpy ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-grid-map-costmap-2d ros-noetic-grid-map-ros ros-noetic-grid-map-filters ros-noetic-grid-map-visualization ros-noetic-tf2-tools pcl-tools
```

## Create the directories

```bash
mkdir -p /home/$USER/dcrobot_ws/src
cd /home/$USER/dcrobot_ws/
```


### Initialize the Catkin workspace
```bash
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release
```

### Navigate to the directory of `src` to clone the `Projeto1 - GrupoB`

```bash
cd /home/$USER/dcrobot_ws/src
git clone https://github.com/vivaldini/dcrobot
git clone https://github.com/MatosAugusto/projeto1_grupoB.git
```

### Build the project
```bash
cd /home/$USER/dcrobot_ws/
catkin build
roscore
```

### Source your catkin workspace
```bash
source /home/$USER/dcrobot_ws/devel/setup.bash
```

### Editing the path planning file

You **must** replace $USER in the line 230 by your linux user name
Also, you must set the start and end point, in the same line

### Launch the .launch files 

You have to run the following commands on your Linux terminal

```bash
/home/$USER/dcrobot_ws/src/dcrobot/mobile_rob_dev_sim/launch

roslaunch gazebo.launch
```
Open a new terminal, because this command occupies the terminal.
Run the path planning

```bash
/home/$USER/dcrobot_ws/src/path_planning/src/scripts

rosrun path_planning path_planning.py
```
After you run this, it will open a window showing the map and the path
You have to close the window to the robot start moving
As said before, the robot, unfortunately, will not follow the right path
