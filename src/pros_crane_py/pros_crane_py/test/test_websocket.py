import asyncio
import websockets
import json

class WS:
    def __init__(self, uri):
        self.uri = uri
    
    async def connect(self):
        self.connection = await websockets.connect(self.uri)
        print("Connected to the server")
        return self.connection

    async def subscribe_to_topic(self, topic, type_msg):
        subscribe_message = json.dumps({
            "op": "subscribe",
            "topic": topic,
            "type": type_msg
        })
        await self.connection.send(subscribe_message)
        print(f"Subscribed to {topic} with type {type_msg}")
    
    async def message_handler(self):
        while True:
            message = await self.connection.recv()
            print(f"Received message: {message}")

async def main():
    ws_instance = WS("ws://localhost:9090")
    connection = await ws_instance.connect()  # Connect to the WebSocket server
    await ws_instance.subscribe_to_topic("/crane", "std_msgs/msg/String")
    await ws_instance.subscribe_to_topic("/joint_trajectory_point", "trajectory_msgs/msg/JointTrajectoryPoint")
    await ws_instance.message_handler()  # Handle incoming messages

# Start the asyncio event loop
asyncio.run(main())
