# Guia da Classe MAV

Aqui pode ser encontrada a documentação completa da classe MAV. 

## Sobre a classe

A classe MAV tem o objetivo de compilar as principais funções de controle de drones, usando ROS. O **arquivo original** pode ser encontrado no package 
viscon em ```viscon/src/viscon``` .

## Mensagens

### Posição 

#### Posição local
        self.drone_pose = PoseStamped()
        self.goal_pose = PoseStamped()
Mensagens do tipo ```geometry_msgs```, usadas na publicão e recepção de posições locais. 

Documentação oficial: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html

#### Posição global
        self.global_pose = NavSatFix()
Mensagem do tipo ```sensor_msgs```, usada para receber posições golbais.

Documentação oficial: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html

        self.gps_target = GeoPoseStamped()
Mensagem do tipo ```geographic_msgs```, usada na publicação de posições globais. 

Documentação oficial: http://docs.ros.org/jade/api/geographic_msgs/html/msg/GeoPoseStamped.html

### Velocidade 
        self.goal_vel = TwistStamped()
Mensagem do tipo ```geometry_msgs```, usada na publicação de velocidades. 

Documentação oficial: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html

### Mensagens de estado 
        self.drone_state = State()
Mensagem do tipo ```mavros_msgs```, usada para receber os estados do drone.

Documentação oficial: http://docs.ros.org/melodic/api/mavros_msgs/html/msg/State.html

## Funções

### Funções "set"
São usadas para mudar instãncias do drone diretamente. 

#### ```set_position```
Configura posições locais em x, y e z.
#### ```set_vel```
Configura velocidades lineares em x, y e z e velocidades angulares em x, y e z.
#### ```set_mode```
Configura o modo/estado do drone para OFFBOARD, e faz uma checagem que garante que o estado é mantido em condições normais. 

### Funções de movimento
São responsáveis por traçar trajetórias, utilizando, para isso, as funções "set". 
#### ```takeoff```
Configura a posição do drone para uma altura determinada e arma o drone, se ele já não estiver armado. A trajetória de decolagem é 
parametrizada na prórpia classe, por posição, com um polinômio calculado, em que as velocidades inicial e final são nulas e a aceleração cresce e decresce de forma suave.  
#### ```RTL```
Responsável por fazer o drone retornar para as coordenadas locais (0, 0, 0), de onde foi feita a decolagem e fazer uma checagem, a partir das 
mensagens recebidas de ```ExtendedState```, se o drone está pousado, para desarmá-lo em seguida. 
#### ```hold```
Responsável por manter o drone estático por um tempo determinado.
#### ```land```
Responsável por pousar o drone na posição atual e fazer uma checagem, a partir das mensagens recebidas de ```ExtendedState```, 
se o drone está pousado, para desarmá-lo em seguida. A trajetória é parametrizada da mesma forma que na função ```takeoff```. 
#### ```disarm```
Desarma o drone, checando se o drone está pousado para que seja desarmado. 
#### ```go_gps_target```
Responsável por levar o drone a uma posição global determinada. Configura as posições globais na própria função, com latitude, longitude, altitude, orientação em x, y e z 
e velocidade angular em z (yaw). 

A trajetória é parametrizada utilizando posições e por meio de controle proporcional, em que a constante de controle do movimento recebe valores crescentes até atingir uma constante determinada. Além disso, na parametrização, é utilizada a biblioteca ```LatLon``` do Python, para conversão de distâncias em latitude e longitude para distâncias metrificadas. 
#### ```set_altitude```
Mantém o drone na mesma posição e configura uma altura determinada, sem pousá-lo. A trajetória é parametrizada da mesma forma que na função ```takeoff```. 
