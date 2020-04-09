from pythonosc import dispatcher
from pythonosc import osc_server
from pythonosc import udp_client
import threading
import time
from typing import List, Any

try:
    from ipywidgets import interact, interactive, interact_manual
    import ipywidgets as widgets
except:
    print("Error importin ipywidgets. Notebook widgets not available")

class AppConnection(object):
    def __init__(self, pserver, handshake_server_addr: str = "127.0.0.1",
                 handshake_server_port: int = 16987, listener_first_port: int = 14001):
        
        self.pserver = pserver
        self.handshakeServerAddr = handshake_server_addr
        self.handshakeServerPort = handshake_server_port
        self.listenerFirstPort = listener_first_port
        self.client = udp_client.SimpleUDPClient(self.handshakeServerAddr, self.handshakeServerPort)
        
        self.d = dispatcher.Dispatcher()
        self.d.map("/requestListenerInfo", self.register_handler)
        self.d.map("/registerListener", self.register_listener)
#         self.d.map("/*", self.message_handler)
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

    def get_listener_info(self):
        PRINT("LISTENER INFO")
        pass;
        
            
    def register_handler(self, address: str, *args: List[Any]):
        print("Sending registering as listener request")
        self.client.send_message("/registerListener", (self.pserver.ip, self.pserver.port))
        self.client.send_message("/requestListenerInfo", (self.handshakeServerAddr, self.listenerFirstPort))
        
    def register_listener(self, address: str, *args: List[Any]):
        self.pserver.add_listener(args[0], args[1])
        
#     def message_handler(self, address: str, *args: List[Any]):
#         print("Unhandled command [{0}] ~ {1}".format(address, args[0]))

    def server_thread_function(self, ip: str, port: int):
#         print("Starting on port " + str(port))
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
        
        self._interactive_widget = None
        self.observers = []
        self._value_callbacks = []
        
    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self.set_value(value)
            
    def set_value(self, value):
        # This assumes we are never primary application, and
        # we don't relay repetitions. This stops feedback,
        # but means if this is the primary app, some things
        # might not work as expected.
#         print("Got " + str(value))
        if not self._value == value:
            self._value = value
            for o in self.observers:
                o.send_parameter_value(self)

            if self._interactive_widget:
                self._interactive_widget.children[0].value = value
        for cb in self._value_callbacks:
            cb(value)
        
    def set_from_internal_widget(self, value):
        self._value = value
        for o in self.observers:
            o.send_parameter_value(self)
        for cb in self._value_callbacks:
            cb(value)
        
    def get_full_address(self):
        # TODO sanitize names
        addr = "/"
        if not self.prefix == "":
            addr += self.prefix + "/"
        if not self.group == "":
            addr += self.group + "/"
        addr += self.name    
        return addr
    
    def interactive_widget(self):
        self._interactive_widget = interactive(self.set_from_internal_widget,
                value=widgets.FloatSlider(
                value=self._value,
                min=self.minimum,
                max=self.maximum,
                description=self.name,
                disabled=False,
                continuous_update=True,
                orientation='horizontal',
                readout=True,
                readout_format='.3f',
            ));
        return self._interactive_widget
    
    def register_callback(self, f):
        self._value_callbacks.append(f)
        

class ParameterServer(object):
    def __init__(self, ip: str = "localhost", start_port: int = 9011):
        self.ip = ip
        self.port = start_port
        self.parameters = []
        self.listeners = []
        self.dispatcher = dispatcher.Dispatcher()
        
        self.start()
        
        self.app_connection = AppConnection(self)
        
        
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
        print("Register listener " + ip + ":" + str(port))
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
        