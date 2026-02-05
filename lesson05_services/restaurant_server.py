#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_interfaces.srv import TakeOrder
# Import the custom TakeOrder service we just defined


class RestaurantServer(Node):
    def __init__(self):
        super().__init__('restaurant_server')
        
        # CREATE THE SERVICE
        # create_service() takes 3 arguments:
        #   1. Service type (TakeOrder)
        #   2. Service name ('/take_order')
        #   3. Callback function (handle_order)
        
        self.srv = self.create_service(
            TakeOrder,
            '/take_order',
            self.handle_order
        )
        
        self.get_logger().info('🍽️  Restaurant is OPEN! Waiting for orders...')
    
    def handle_order(self, request, response):
        # SERVER CALLBACK
        # Called when a client sends a request
        # request: Contains the data the client sent
        # response: Object we fill with the response data
        
        self.get_logger().info(f'📋 Incoming request: {request.quantity}x {request.item_name}')
        
        # Business logic: Check if we have the item
        if request.item_name.lower() == 'burger':
            response.accepted = True
            response.message = f'✅ Delicious! {request.quantity} burgers coming up!'
        
        elif request.item_name.lower() == 'salad':
            response.accepted = True
            response.message = '✅ Healthy choice! Salad on the way.'
        
        else:
            response.accepted = False
            response.message = f'❌ Sorry, we do not serve {request.item_name}.'
        
        self.get_logger().info(f'📤 Sending response: {response.message}')
        
        return response
        # MUST return the response object


def main(args=None):
    rclpy.init(args=args)
    node = RestaurantServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
