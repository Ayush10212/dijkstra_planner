import rclpy
from rclpy.node import Node
from queue import PriorityQueue
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseStamped

class DijkstraPlannerNode(Node):
    def __init__(self):
        super().__init__('dijkstra_node')
        self.get_logger().info('Dijkstra Planner Node Started')
        
        # Publishers/Subscribers
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Parameters
        self.declare_parameter('grid', [0,0,1,0, 1,0,1,0, 0,0,0,0, 0,1,1,0])
        self.declare_parameter('rows', 4)
        self.declare_parameter('cols', 4)
        self.declare_parameter('start', [0,0])
        self.declare_parameter('goal', [3,3])
        
        # Initialize grid
        grid_flat = self.get_parameter('grid').value
        self.rows = self.get_parameter('rows').value
        self.cols = self.get_parameter('cols').value
        self.grid = np.array(grid_flat).reshape(self.rows, self.cols)
        
        # Run initial planning
        start = tuple(self.get_parameter('start').value)
        goal = tuple(self.get_parameter('goal').value)
        path = self.dijkstra(start, goal)
        self.get_logger().info(f"Initial path: {path}")

    def dijkstra(self, start, goal):
        pq = PriorityQueue()
        pq.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while not pq.empty():
            _, current = pq.get()
            
            if current == goal:
                break
                
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = current[0]+dx, current[1]+dy
                if 0 <= nx < self.rows and 0 <= ny < self.cols and self.grid[nx,ny] == 0:
                    new_cost = cost_so_far[current] + 1
                    if (nx,ny) not in cost_so_far or new_cost < cost_so_far[(nx,ny)]:
                        cost_so_far[(nx,ny)] = new_cost
                        pq.put((new_cost, (nx,ny)))
                        came_from[(nx,ny)] = current
        
        # Reconstruct path
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        
        return path

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
