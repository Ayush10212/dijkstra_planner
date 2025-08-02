import rclpy
from rclpy.node import Node
from queue import PriorityQueue

class DijkstraPlannerNode(Node):
    def __init__(self):
        super().__init__('dijkstra_node')
        self.get_logger().info('Dijkstra Planner Node Started')
        
        # Initialize grid from parameters
        self.declare_parameter('grid', [0,0,1,0, 1,0,1,0, 0,0,0,0, 0,1,1,0])
        self.declare_parameter('rows', 4)
        self.declare_parameter('cols', 4)
        
        grid_flat = self.get_parameter('grid').value
        rows = self.get_parameter('rows').value
        cols = self.get_parameter('cols').value
        
        self.grid = [grid_flat[i*cols:(i+1)*cols] for i in range(rows)]
        self.start = (0, 0)
        self.goal = (3, 3)
        
        path = self.dijkstra(self.start, self.goal)
        self.get_logger().info(f"Path: {path}")

    def dijkstra(self, start, goal):
        # Dijkstra's algorithm implementation
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
