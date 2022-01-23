import numpy as np
import osmnx as ox
import networkx as nx
import gurobipy as gp
from gurobipy import GRB
import matplotlib.pyplot as plt

class Instancia:
    def __init__(self,nombre):
        archivo = open(nombre, "r")
        self.n = int(archivo.readline().rstrip())
        self.deposito = 0
        self.coordenadas = [tuple(map(float,archivo.readline().rstrip().split(" "))) for _ in range(self.n)]
        archivo.close()
        self.coordenadas = {int(n):(x,y) for n,x,y in self.coordenadas}
        self.__inicializarInstancia()

    def __inicializarInstancia(self):
        n = self.n
        graph_area = self.coordenadas[self.deposito]
        G = ox.graph_from_point(graph_area, network_type='drive',dist = 7000)
        G = ox.add_edge_speeds(G)
        G = ox.add_edge_travel_times(G)
        distancias = np.zeros((n,n),dtype=float)
        for i in range(n):
            for j in range(n):
                if i == j:
                    distancias[i,j] = np.inf
                else:
                    t1 = self.coordenadas[i]
                    t2 = self.coordenadas[j]
                    origen = ox.get_nearest_node(G, t1)
                    destino =ox.get_nearest_node(G, t2)
                    aux = nx.shortest_path_length(G, origen, destino, weight='length')
                    distancias[i,j] = aux
        self.G = G
        self.__distancias = distancias
        
    def resolver(self):
        n = self.n
        N = [i for i in range(n)]
        arcos, costo = gp.multidict({(i,j): self.distancia(i,j) for i in range(n) for j in range(n) if i!=j})

        m = gp.Model('Traveling Drunkman Problem')
        x = m.addVars(arcos, name = "x",vtype=gp.GRB.BINARY)
        g = m.addVars(arcos, name = "g",vtype=gp.GRB.CONTINUOUS)

        unaSalida = m.addConstrs((x.sum(i,'*') == 1 for i in N), name='unaSalida')
        unaEntrada = m.addConstrs((x.sum('*',j) == 1 for j in N), name='unaEntrada')
        deposito = m.addConstr((g.sum(0,'*') == 0), name='deposito')
        orden = m.addConstrs((g.sum(i,'*') - g.sum('*',i) == 1 for i in N[1:]), name = "orden")
        gmaximo = m.addConstrs((g[i,j] <= (n-1)*x[i,j] for i,j in arcos), name = "gmaximo")
        gpositivo = m.addConstrs((g[i,j] >= 0 for i,j in arcos), name = "gpositivo")
        xpositivo = m.addConstrs((g[i,j] >= 0 for i,j in arcos),name = "xpositivo")
        
        m.setObjective(x.prod(costo), GRB.MINIMIZE)
        m.optimize()
        obj = m.getObjective()
        self.objetivo = obj.getValue()
        self.solucion = [(i,j) for i,j in arcos if x[i,j].x >0.9]
        return None

    def distancia(self,i,j):
        return self.__distancias[i,j]

    def visualizarCiudad(self):
        arcosSolucion = self.solucion
        G = self.G
        rutas = [ox.shortest_path(G, ox.get_nearest_node(G, self.coordenadas[i]), ox.get_nearest_node(G, self.coordenadas[j])) for i,j in arcosSolucion]
        fig, ax = ox.plot_graph_routes(self.G, rutas, "r", route_linewidth=6, node_size=0)

def main():
    TDP = Instancia("alcohol.tsp")
    TDP.resolver()
    TDP.visualizarCiudad()

if __name__ == "__main__":
    main()