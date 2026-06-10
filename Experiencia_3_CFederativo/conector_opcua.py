from buildingClient import MyClient  # Importar la clase MyClient desde buildingClient.py


class OPCConector(MyClient):
    def __init__(self, ip, port):
        self.url = f"opc.tcp://{ip}:{port}/freeopcua/server/"
        self.client = MyClient(self.url)
        self.last_step = -1  # Variable para almacenar el último Sim_Step leído, inicia en -1 para asegurar que se lea el primer paso de simulación (Sim_Step 0)

    def conectar(self):
        self.client.connect()

    def desconectar(self):
        self.client.disconnect()

    def wait_for_next_step(self):
        self.last_step = self.client.wait_for_next_step(self.last_step)
        return self.last_step

    def obtener_temperaturas(self):
        temperatura_habitaciones = []
        for i in range(self.client.num_rooms):
            temp = self.client.T_rooms[f'T_room{i + 1}'].get_value()  # se usa i +1 porque las habitaciones están indexadas desde 1 (T_room1, T_room2, etc.)
            temperatura_habitaciones.append(temp)
        return temperatura_habitaciones

    def obtener_temperatura_exterior(self):
        return self.client.T_outdoor.get_value()

    def obtener_voltajes(self):
        voltajes = []
        for i in range(self.client.num_rooms):
            volt = self.client.voltages[f'Volt_room{i + 1}'].get_value()  # se usa i +1 porque las habitaciones están indexadas desde 1 (Volt_room1, Volt_room2, etc.)
            voltajes.append(volt)
        return voltajes

    def fijar_voltaje(self, id, voltaje):
        self.client.voltages[f'Volt_room{id}'].set_value(voltaje)  # se usa id -1 porque las habitaciones están indexadas desde 1 (Volt_room1, Volt_room2, etc.)

    def fijar_todos_voltajes(self, voltajes):
        for i, voltaje in enumerate(voltajes):
            self.client.voltages[f'Volt_room{i + 1}'].set_value(voltaje)  # se usa i +1 porque las habitaciones están indexadas desde 1 (Volt_room1, Volt_room2, etc.)

from buildingClient import MyClient


class OPCConector(MyClient):

    def __init__(self, ip, port):

        self.url = f"opc.tcp://{ip}:{port}/freeopcua/server/"

        self.client = MyClient(self.url)

        self.last_step = -1

    def conectar(self):

        self.client.connect()

    def desconectar(self):

        self.client.disconnect()

    def wait_for_next_step(self):

        self.last_step = self.client.wait_for_next_step(
            self.last_step
        )

        return self.last_step

    def obtener_temperaturas(self):

        temperatura_habitaciones = []

        for i in range(self.client.num_rooms):

            temp = self.client.T_rooms[
                f'T_room{i + 1}'
            ].get_value()

            temperatura_habitaciones.append(temp)

        return temperatura_habitaciones

    def obtener_temperatura_exterior(self):

        return self.client.T_outdoor.get_value()

    def obtener_voltajes(self):

        voltajes = []

        for i in range(self.client.num_rooms):

            volt = self.client.voltages[
                f'Volt_room{i + 1}'
            ].get_value()

            voltajes.append(volt)

        return voltajes

    def fijar_voltaje(self, id, voltaje):

        self.client.voltages[
            f'Volt_room{id}'
        ].set_value(voltaje)

    def fijar_todos_voltajes(self, voltajes):

        for i, voltaje in enumerate(voltajes):

            self.client.voltages[
                f'Volt_room{i + 1}'
            ].set_value(voltaje)


# ============================================================
# SOLO PARA TESTING
# ============================================================

if __name__ == "__main__":

    ip = "192.168.1.142"

    puerto = 4840

    cliente = OPCConector(ip, puerto)

    cliente.conectar()

    try:

        while True:

            step = cliente.wait_for_next_step()

            temps = cliente.obtener_temperaturas()

            tout = cliente.obtener_temperatura_exterior()

            volts = cliente.obtener_voltajes()

            print("\n================================")

            print(f"Sim Step: {step}")

            for i, temp in enumerate(temps):

                print(f"Habitación {i+1}: {temp:.2f} °C")

            print(f"Exterior: {tout:.2f} °C")

            print(f"Voltajes: {volts}")

            print("================================")

    except KeyboardInterrupt:

        print("Deteniendo cliente...")

    finally:

        cliente.desconectar()
        