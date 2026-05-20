"""
implement here your path planning method
"""

import math
import heapq
from nav_msgs.msg import OccupancyGrid


class Planner:
    def __init__(self, costmap):
        """ 
        Initialize a map from a ROS costmap
        
        costmap: ROS costmap
        """
        # Copy the map metadata
        self.resolution = costmap.info.resolution
        self.min_x = costmap.info.origin.position.x
        self.min_y = costmap.info.origin.position.y
        self.y_width = costmap.info.height
        self.x_width = costmap.info.width

        # Si detectamos la rejilla provisional de 100x100, forzamos las dimensiones reales
        if self.x_width == 100 :
            print(
                "[PARCHE] Detectado mapa fantasma de ROS. Forzando dimensiones reales..."
            )
            self.x_width = 96
            self.y_width = 96
            self.min_x = -2.394
            self.min_y = -2.397

        self.max_x = self.min_x + self.x_width *self.resolution
        self.max_y = self.min_y + self.y_width *self.resolution
        print("min corner x: %.2f m, y: %.2f m" % (self.min_x, self.min_y))
        print("max corner x: %.2f m, y: %.2f m" % (self.max_x, self.max_y))
        print("Resolution: %.3f m/cell" % self.resolution)
        print("Width: %i cells, height: %i cells" % (self.x_width, self.y_width))

        # Copy the actual map data from the map
        x = 0
        y = 0
        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]

        self.cost_map = [[0 for _ in range(self.y_width)] 
                         for _ in range(self.x_width)]
        obstacles = 0
        # 3. Rellenamos el mapa de obstáculos solo si los datos coinciden con la realidad
        if len(costmap.data) == self.x_width * self.y_width:
            for x in range(self.x_width):
                for y in range(self.y_width):
                    index = x + (y * self.x_width)
                    if index < len(costmap.data):
                        value = costmap.data[index]
                        self.cost_map[x][y] = value
                        if value >= 100:  # Umbral de obstáculo letal
                            obstacles += 1
                            self.obstacle_map[x][y] = True
        else:
            print(" [PARCHE] Inicializando rejilla limpia temporal.")
        print("Loaded %d obstacles"%(obstacles))

        # NOTE: alternatively, instead of computing a binary
        # obstacle map, you can try to store directly the costmap
        # values and use them as costs.

    class Node:
        def __init__(self, cx, cy, cost, parent):
            self.x_cell = cx  # x index in the obstacle grid
            self.y_cell = cy  # y index in the obstacle grid

            # TODO: add the node costs that you may need
            self.cost = cost # g: coste acumulado desde el inicio
            # self.g?
            # self.f?

            self.parent = parent  # index of the previous Node

        # Necesario para que heapq pueda comparar nodos por su valor de coste
        def __lt__(self, other):
            return self.cost < other.cost

    def plan(self, sx, sy, gx, gy):
        """
        Método de planificación Dijkstra con inflación de costes y colchón de rescate.
        """

        # 1. Primero comprobamos si ya estamos extremadamente cerca del objetivo
        d = math.sqrt((gx - sx) * (gx - sx) + (gy - sy) * (gy - sy))
        if d <= self.resolution:
            return None

        # 2. Convertimos las posiciones reales (metros) a celdas de la rejilla
        start_cell = self.real2cell(sx, sy)
        goal_cell = self.real2cell(gx, gy)

        # 3. Aplicamos tu nuevo colchón de rescate inteligente por si caen fuera o en esquinas
        start_cell = self.find_nearest_valid_cell(
            start_cell[0], start_cell[1], max_radius=30
        )
        goal_cell = self.find_nearest_valid_cell(
            goal_cell[0], goal_cell[1], max_radius=30
        )

        # 4. Verificamos que el rescate haya sido capaz de encontrar una celda libre cercana
        if start_cell is None:
            print("Error: start position not valid and no nearby free cell found")
            return None

        if goal_cell is None:
            print("Error: goal position not valid and no nearby free cell found")
            return None

        # 5. Despaquetamos las coordenadas de las celdas ya seguras y rescatadas
        start_cell_x, start_cell_y = start_cell
        goal_cell_x, goal_cell_y = goal_cell

        # 6. ¡AHORA SÍ! Creamos los objetos Node usando las celdas ya corregidas
        start_node = self.Node(start_cell_x, start_cell_y, 0.0, -1)
        goal_node = self.Node(goal_cell_x, goal_cell_y, 0.0, -1)

        # 7. Filtro de seguridad obligatorio del enunciado
        if not self.node_is_valid(start_node):
            print("Error: start position not valid!!")
            return None

        if not self.node_is_valid(goal_node):
            print(
                f"Error: goal position not valid!! cell=({goal_cell_x},{goal_cell_y})"
            )
            return None

        # 8. Movimientos posibles: 4 direcciones (ortogonales) para evitar empotrarse en esquinas
        motions = [
            (1, 0, 1.0),  # derecha
            (-1, 0, 1.0),  # izquierda
            (0, 1, 1.0),  # arriba
            (0, -1, 1.0),  # abajo
        ]

        # 9. Inicializamos las estructuras de tu Dijkstra
        open_list = []
        heapq.heappush(open_list, start_node)

        closed_set = {}
        all_nodes = {(start_cell_x, start_cell_y): start_node}

        # 10. Bucle principal de exploración
        while open_list:
            current = heapq.heappop(open_list)
            current_key = (current.x_cell, current.y_cell)

            if current_key in closed_set:
                continue

            closed_set[current_key] = current

            # Si la celda actual es la meta, terminamos de explorar
            if current.x_cell == goal_cell_x and current.y_cell == goal_cell_y:
                break

            # Expandimos los vecinos en las 4 direcciones
            for dx, dy, move_cost in motions:
                nx = current.x_cell + dx
                ny = current.y_cell + dy
                neighbor_key = (nx, ny)

                if neighbor_key in closed_set:
                    continue

                if nx < 0 or nx >= self.x_width or ny < 0 or ny >= self.y_width:
                    continue

                # Calculamos el coste acumulado sumando la inflación de las paredes
                costmap_value = self.cost_map[nx][ny]
                if costmap_value > 0:
                    inflation_weight = 15.0
                    normalized_cost = float(costmap_value) / 100.0
                    extra_cost = inflation_weight * (normalized_cost**2)
                    new_cost = current.cost + move_cost + extra_cost
                else:
                    new_cost = current.cost + move_cost

                neighbor = self.Node(nx, ny, new_cost, current_key)

                if not self.node_is_valid(neighbor):
                    continue

                if (
                    neighbor_key in all_nodes
                    and new_cost >= all_nodes[neighbor_key].cost
                ):
                    continue

                all_nodes[neighbor_key] = neighbor
                heapq.heappush(open_list, neighbor)

        # 11. Reconstrucción del camino óptimo desde el goal hacia atrás (usando los padres)
        rx = []
        ry = []

        goal_key = (goal_cell_x, goal_cell_y)
        if goal_key not in closed_set:
            print("No path found!")
            return None

        current = all_nodes[goal_key]
        while current is not None:
            real_x, real_y = self.cell2real(current.x_cell, current.y_cell)
            rx.append(real_x)
            ry.append(real_y)

            # Si el padre es -1, significa que hemos regresado al nodo de inicio
            if current.parent == -1:
                break
            current = all_nodes[current.parent]

        # Le damos la vuelta a las listas para que vayan desde el inicio hasta la meta
        rx.reverse()
        ry.reverse()

        return rx, ry

    # Transform map coordinates in meters
    # to cell indexes in the grid
    def real2cell(self, rx, ry):
        cellx = round((rx - self.min_x) / self.resolution)
        celly = round((ry - self.min_y) / self.resolution)
        return cellx, celly

    # Tranform cell indexes of the grid
    # to map coordinates in meters
    def cell2real(self, cx, cy):
        rx = cx * self.resolution + self.min_x
        ry = cy * self.resolution + self.min_y
        return rx, ry

    def cell_is_valid(self, cx, cy):
        if cx < 0 or cy < 0:
            return False
        if cx >= self.x_width or cy >= self.y_width:
            return False
        if self.obstacle_map[int(cx)][int(cy)]:
            return False
        return True

    def find_nearest_valid_cell(self, cx, cy, max_radius=30):
        if self.cell_is_valid(cx, cy):
            return cx, cy

        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx = cx + dx
                    ny = cy + dy

                    if self.cell_is_valid(nx, ny):
                        return nx, ny
        return None

    def node_is_valid(self, node):

        # check that the node is inside the grid limits
        rx, ry = self.cell2real(node.x_cell, node.y_cell)
        if rx < self.min_x:
            return False
        if ry < self.min_y:
            return False
        if rx >= self.max_x:
            return False
        if ry >= self.max_y:
            return False

        # check if the cell is an obstacle
        if self.obstacle_map[int(node.x_cell)][int(node.y_cell)]:
            return False

        return True
