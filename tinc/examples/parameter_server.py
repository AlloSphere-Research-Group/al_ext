from pythonosc import dispatcher
from pythonosc import osc_server
from pythonosc import udp_client
import threading
import time
from typing import List, Any

class AppConnection(object):
    def __init__(self, pserver_port : int, handshake_server_addr: str = "127.0.0.1",
                 handshake_server_port: int = 16987, listener_first_port: int = 14000):
        
        self.pserver_port = pserver_port
        self.handshakeServerAddr = handshake_server_addr
        self.handshakeServerPort = handshake_server_port
        self.listenerFirstPort = listener_first_port
        self.client = udp_client.SimpleUDPClient(self.handshakeServerAddr, self.handshakeServerPort)
        
        self.d = dispatcher.Dispatcher()
        self.d.map("/requestListenerInfo", self.register_handler)
        self.d.map("/*", self.message_handler)
        self.start()
        
    def __del__(self):
        self.stop()
        
    def start(self):

        self.running = True
        self.x = threading.Thread(target=self.server_thread_function, args=(self.handshakeServerAddr, self.listenerFirstPort))
        self.x.start()
        
        time.sleep(0.1)
        self.client.send_message("/handshake", self.listenerFirstPort)
        
    def stop(self):
        if self.running:
            self.running = False
            self.x.join()
            self.server = None

    def register_handler(self, address: str, *args: List[Any]):
        print("Registering listener")
        self.client.send_message("/registerListener", self.pserver_port)

    def message_handler(self, address: str, *args: List[Any]):
        print("Unhandled command [{0}] ~ {1}".format(address, args[0]))

    def server_thread_function(self, ip: str, port: int):
        print("Starting on port " + str(port))
        self.server = osc_server.ThreadingOSCUDPServer(
          (ip, port), self.d)
        self.server.timeout = 0.1
        print("Serving on {}".format(self.server.server_address))
        while self.running:
            self.server.handle_request()
        print("Closed command server")


class Parameter(object):
    def __init__(self, name: str, group: str, default: float, prefix: str = "", minimum: float = -99999.0, maximum: float = 99999.0):
        self._value :str = 0.0
        self.name = name
        self.group = group
        self.default = default
        self.prefix = prefix
        self.minimum = minimum
        self.maximum = maximum
        
        self.observers = []
        
    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value
        for o in self.observers:
            o.send_parameter_value(self)
        
    def get_full_address(self):
        # TODO sanitize names
        addr = "/"
        if not self.prefix == "":
            addr += self.prefix + "/"
        if not self.group == "":
            addr += self.group + "/"
        addr += self.name    
        return addr

class ParameterServer(object):
    def __init__(self, ip: str = "localhost", start_port: int = 9011):
        self.ip = ip
        self.port = start_port
        self.parameters = []
        self.listeners = []
        self.dispatcher = dispatcher.Dispatcher()
        
        self.start()
        
        self.app_connection = AppConnection(self.port)
        
    def __del__(self):
        self.stop()
        
    def start(self):
        self.running = True
        self.x = threading.Thread(target=self.server_thread_function, args=(self.ip, self.port))
        self.x.start()
        
    def stop(self):
        self.app_connection.stop()
        self.running = False
        self.x.join()
        
    def monitor_server(self, timeout: float = 30):
        timeAccum = 0.0
        while timeAccum < timeout or timeout == 0:
            timeAccum += 0.1
            time.sleep(0.1)
#             yield
        
    def register_parameter(self, p):
        p.observers.append(self)
        self.parameters.append(p)
        
        self.dispatcher.map(p.get_full_address(), self.set_parameter_value, p)
        
        
    def register_parameters(self, params):
        for p in params:
            self.register_parameter(p)
        
    def set_parameter_value(self, addr, p: str, *args):
        # TODO there should be a way to select whether to relay duplicates or not, perhaps depending on a role
        if not p[0].value == args[0]:
            p[0].value = args[0]
#         print("got parameter message " + str(addr) + str(args))
        
    def add_listener(self, ip: str, port: int):
        
        self.listeners.append(udp_client.SimpleUDPClient(ip, port))
        
    def server_thread_function(self, ip: str, port: int):
        self.server = osc_server.ThreadingOSCUDPServer(
          (ip, port), self.dispatcher)
        self.server.timeout = 1.0
        print("Parameter server started on {}".format(self.server.server_address))
        while self.running:
            self.server.handle_request()
            
        self.server.server_close()
        print("Closed parameter server")
        
    def send_parameter_value(self, p):
        for l in self.listeners:
#             print("Sending " + p.get_full_address())
            l.send_message(p.get_full_address(), float(p.value))
        