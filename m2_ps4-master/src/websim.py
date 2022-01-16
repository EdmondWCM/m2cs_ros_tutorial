#!/usr/bin/env python3

import asyncio
import json
import rospy
import threading
import websockets

from m2_ps4.msg import Ps4Data
from m2_ps4.srv import *


class WebsimNode:
    def __init__(self):
        rospy.init_node("ps4_websim")

        port = rospy.get_param("port", 22951)

        self.kill_current_session = None
        self.thread_lock = threading.Lock()

        class WsThread(threading.Thread):
            def __init__(self, cb, port):
                super().__init__(daemon=True)
                self.cb = cb
                self.port = port
                self.loop = None

            def run(self):
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
                server = websockets.serve(self.cb, "0.0.0.0", self.port)
                self.loop.run_until_complete(server)
                print(f"Serving on 0.0.0.0:{port}")
                self.loop.run_forever()

            def kill(self):
                if self.loop is not None:
                    self.loop.stop()

        self.conn_id = 0

        self.ws_thread = WsThread(self.ws, port)
        self.ws_thread.start()

        self.last_data = Ps4Data()
        self.message_queue = []

        topic = rospy.get_param("~topic", "/input/ps4_data")
        self.data_pub = rospy.Publisher(topic, Ps4Data, queue_size=1)

        rospy.Service("/set_led", SetRgb, self.set_led)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.heartbeat()
            rate.sleep()

    def heartbeat(self):
        self.data_pub.publish(self.last_data)

    def kill(self):
        if self.kill_current_session is not None:
            self.kill_current_session = None
        self.ws_thread.kill()

    async def ws(self, ws: websockets.WebSocketServerProtocol, _path):
        conn_id = self.conn_id
        self.conn_id += 1

        with self.thread_lock:
            kill_current_session = self.kill_current_session
            my_lock = ws.close
            self.kill_current_session = my_lock

        if kill_current_session is not None:
            print("Killing previous session")
            await self.kill_current_session()

        print(f"Connection {conn_id}: Connected from {ws.remote_address}")
        while not ws.closed and self.kill_current_session is my_lock:
            with self.thread_lock:
                queue = self.message_queue.copy()
                self.message_queue = []
            for msg in queue:
                await ws.send(json.dumps(msg))
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.01)
                data = json.loads(msg)
                if data["type"] == "input":
                    for k, v in data["data"].items():
                        setattr(self.last_data, k, v)
            except asyncio.TimeoutError:
                pass

        self.kill_current_session = None
        if not ws.closed:
            await ws.close()

    def set_led(self, req):
        with self.thread_lock:
            self.message_queue.append(
                {
                    "type": "led",
                    "r": req.rgb_sequence[0].red,
                    "g": req.rgb_sequence[0].green,
                    "b": req.rgb_sequence[0].blue,
                }
            )

        return SetRgbResponse()


if __name__ == "__main__":
    WebsimNode()
