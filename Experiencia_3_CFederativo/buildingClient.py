from opcua import ua, Client
import threading
import time

def funcion_handler(node, val):
    key = node.get_parent().get_display_name().Text
    print('key: {} | val: {}'.format(key, val))

class SubHandler(object):

    """
    Subscription Handler. To receive events from server for a subscription
    data_change and event methods are called directly from receiving thread.
    Do not do expensive, slow or network operation there. Create another
    thread if you need to do such a thing
    """

    def datachange_notification(self, node, val, data):
        thread_handler = threading.Thread(target=funcion_handler, args=(node, val))  # Se realiza la descarga por un thread
        thread_handler.start()

    def event_notification(self, event):
        print("Python: New event", event)



class MyClient:
    def __init__(self, direction, namespace=2):
        self.direction = direction
        self.client = Client(direction)
        self.mv = []
        self.dv = []
        self.T = 100 # cantidad de milisegundos para revisar las variables subscritas
        self.namespace = namespace

    def instantiate(self):
        self.root = self.client.get_root_node()
        self.objects = self.client.get_objects_node()
        self.rooms = self.objects.get_child(['{}:Building'.format(self.namespace),'{}:Rooms'.format(self.namespace)])
        self.num_rooms = self.objects.get_child(['{}:Building'.format(self.namespace),'{}:Num_Rooms'.format(self.namespace)]).get_value()

        self.T_rooms = {'T_room{}'.format(i + 1): self.rooms.get_child(['{}:T_room{}'.format(self.namespace, i + 1)]) for i in range(self.num_rooms)}
        self.T_outdoor = self.rooms.get_child(['{}:T_outdoor'.format(self.namespace)])
        self.T_ground = self.rooms.get_child(['{}:T_ground'.format(self.namespace)])
        self.Qghi_rooms = {'Qghi_room{}'.format(i + 1): self.rooms.get_child(['{}:Qghi_room{}'.format(self.namespace, i + 1)]) for i in range(self.num_rooms)}
        self.Qpk_rooms = {'Qpk_room{}'.format(i + 1): self.rooms.get_child(['{}:Qpk_room{}'.format(self.namespace, i + 1)]) for i in range(self.num_rooms)}

        self.voltages = {'Volt_room{}'.format(i + 1): self.rooms.get_child(['{}:Volt_room{}'.format(self.namespace, i + 1)]) for i in range(self.num_rooms)}
        self.sim_time = self.objects.get_child(['{}:Building'.format(self.namespace),'{}:Sim_Time'.format(self.namespace)])
        self.sim_scale = self.objects.get_child(['{}:Building'.format(self.namespace),'{}:Sim_Scale'.format(self.namespace)])
        self.timestamp = self.objects.get_child(['{}:Building'.format(self.namespace),'{}:Timestamp'.format(self.namespace)])
        
        
        self.sim_step = self.objects.get_child(['{}:Building'.format(self.namespace),'{}:Sim_Step'.format(self.namespace)])
        self.ready_plant = self.objects.get_child(['{}:Building'.format(self.namespace),'{}:ReadyPlant'.format(self.namespace)])
        self.ready_ctrl  = self.objects.get_child(['{}:Building'.format(self.namespace),'{}:ReadyCtrl'.format(self.namespace)])


    def wait_for_next_step(self, last_step, poll_s=0.01, timeout_s= None):
        t0 = time.time()
        while True:
            step = int(self.sim_step.get_value())
            if step != last_step:
                return step

            if timeout_s is not None and (time.time() - t0) > timeout_s:
                raise TimeoutError(f"Timeout esperando nuevo Sim_Step (last_step={last_step})")

            time.sleep(poll_s)

    def set_ready_plant(self, value: bool):
        self.ready_plant.set_value(bool(value))

    def set_ready_ctrl(self, value: bool):
        self.ready_ctrl.set_value(bool(value))


    def connect(self):
        try:
            self.client.connect()
            self.objects = self.client.get_objects_node()
            print('Cliente OPCUA se ha conectado')
            self.instantiate()

        except:
            self.client.disconnect()
            print('Cliente no se ha podido conectar')

    def disconnect(self):
        try:
            self.client.disconnect()
            print('Cliente OPCUA desconectado')
        except Exception as e:
            print(f'Error desconectando: {e}')

if __name__ == '__main__':
    client = MyClient("opc.tcp://localhost:4840/freeopcua/server/", SubHandler)
    client.connect()
