#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_interfaces.srv import TakeOrder
import sys


class RestaurantClient(Node):
    def __init__(self):
        super().__init__('restaurant_client')
        
        # CREATE A CLIENT FOR THE SERVICE
        self.cli = self.create_client(TakeOrder, '/take_order')
        
        # Wait for server to be ready
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for service /take_order...')
        
        self.get_logger().info('✅ Connected to restaurant!')
    
    def send_request(self, item_name, quantity):
        # Create request
        request = TakeOrder.Request()
        request.item_name = item_name
        request.quantity = quantity
        
        self.get_logger().info(f'🍽️  Placing order: {quantity}x {item_name}')
        
        # Send request asynchronously
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.accepted:
                self.get_logger().info(f'✅ {response.message}')
            else:
                self.get_logger().info(f'❌ {response.message}')
        else:
            self.get_logger().error('Service call failed!')


def main(args=None):
    rclpy.init(args=args)
    
    # Get item and quantity from command line
    if len(sys.argv) < 3:
        print("Usage: ros2 run test2_py_pkg restaurant_client <item> <quantity>")
        print("Example: ros2 run test2_py_pkg restaurant_client burger 2")
        return
    
    item = sys.argv[1]          # e.g., 'burger'
    quantity = int(sys.argv[2]) # e.g., 2
    
    client = RestaurantClient()
    client.send_request(item, quantity)


if __name__ == '__main__':
    main()
