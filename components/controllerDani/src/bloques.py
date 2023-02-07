"""
Diccionario que permite generar las reglas para poder codifocar y controlar los estados del problema.
(Sacado a diccionario para facilitar mantenimiento futuro)
"""
reglas = {
    "SM": lambda x: f"SM({x})",
    "S": lambda x,y: f"S({x},{y})",
    "L": lambda x: f"L({x})",
    "C": lambda x: f"C({x})",
    "MV": "MV",
}


class MundoBloques():
    """
    Clase para la gestión lógica del mundo de bloques
    """
    def __init__(self):
        self.estado_inicial = {}
        self.estado_actual = {}
        self.estado_final = {}

    def set_states(inicial, final):
        """
        Inicialización de los estados
        """
        self.estado_inicial = inicial
        self.estado_actual = self.estado_inicial
        self.estado_final = final
    
    def get_estado_actual(self):
        """
        Devuelve el estado lógico actual
        """
        return self.estado_actual

    def coger(bloque):
        """
        PRE: SobreMesa(bloque) && Libre(bloque) && ManoVacia
        DEL: SobreMesa(bloque) && ManoVacia
        ADD: Cogido(bloque)
        """
        if set((reglas["SM"](bloque), reglas["L"](bloque), reglas["MV"])).issubset(estado_actual):
            estado_actual.remove(reglas["SM"](bloque))
            estado_actual.remove(reglas["MV"])
            estado_actual.add(reglas["C"](bloque))
        else:
            print("COGER: No se cumplen las precondiciones")


    def dejar(bloque):
        """
        PRE: Cogido(bloque)
        DEL: Cogido(bloque)
        ADD: SobreMesa(bloque) && ManoVacia
        """
        if set((reglas["C"](bloque))).issubset(estado_actual):
            estado_actual.remove(reglas["C"](bloque))
            estado_actual.add(reglas["SM"](bloque))
            estado_actual.add(reglas["MV"])
        else:
            print("DEJAR: No se cumplen las precondiciones")


    def apilar(bloque, destino):
        """
        PRE: Cogido(bloque) && Libre(destino)
        DEL: Cogido(bloque) && Libre(destino)
        ADD: Sobre(destino, bloque) && && ManoVacia
        """
        if set((reglas["C"](bloque), reglas["L"](destino))).issubset(estado_actual):
            estado_actual.remove(reglas["C"](bloque))
            estado_actual.remove(reglas["L"](destino))
            estado_actual.add(reglas["S"](destino, bloque))
            estado_actual.add(reglas["MV"])
        else:
            print("APILAR: No se cumplen las precondiciones")

    def desapilar(bloque, fuente):
        """
        PRE: Sobre(fuente, bloque) && Libre(bloque) && ManoVacia
        DEL: Sobre(fuente, bloque) && ManoVacia
        ADD: Cogido(bloque) && Libre(fuente)
        """
        if set((reglas["S"](fuente, bloque), reglas["L"](bloque), regas["MV"])).issubset(estado_actual):
            estado_actual.remove(reglas["S"](fuente, bloque))
            estado_actual.remove(reglas["MV"])
            estado_actual.add(reglas["C"](bloque))
            estado_actual.add(reglas["L"](fuente))
        else:
            print("DESAPILAR: No se cumplen las precondiciones")
