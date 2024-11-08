/*
Instrucciones:
En equipos de máximo 3 personas, escribe en C++ un programa que ayude a una empresa que quiere
incursionar en los servicios de internet respondiendo a la situación problema 2.

El programa debe:
1. Leer un archivo de entrada que contiene la información de un grafo representado en forma de una
matriz de adyacencias con grafos ponderados. El peso de cada arista es la distancia en kilómetros entre
colonia y colonia, por donde es factible meter cableado.

El programa debe desplegar cuál es la forma óptima de cablear con fibra óptica conectando colonias de
tal forma que se pueda compartir información entre cuales quiera dos colonias.

2. Debido a que las ciudades apenas están entrando al mundo tecnológico, se requiere que alguien visite
cada colonia para ir a dejar estados de cuenta físicos, publicidad, avisos y notificaciones impresos.
Por eso se quiere saber ¿cuál es la ruta más corta posible que visita cada colonia exactamente una vez y
al finalizar regresa a la colonia origen?

El programa debe desplegar la ruta a considerar, tomando en cuenta que la primera ciudad se le llamará
A, a la segunda B, y así sucesivamente.

3. El programa también debe leer otra matriz cuadrada de N x N datos que representen la capacidad máxima
de transmisión de datos entre la colonia i y la colonia j. Como estamos trabajando con ciudades con una
gran cantidad de campos electromagnéticos, que pueden generar interferencia, ya se hicieron estimaciones
que están reflejadas en esta matriz.

La empresa quiere conocer el flujo máximo de información del nodo inicial al nodo final. Esto debe
desplegarse también en la salida estándar.

Por último,

4. Teniendo en cuenta la ubicación geográfica de varias "centrales" a las que se pueden conectar nuevas
casas, la empresa quiere contar con una forma de decidir, dada una nueva contratación del servicio, cuál
es la central más cercana geográficamente a esa nueva contratación. No necesariamente hay una central
por cada colonia. Se pueden tener colonias sin central, y colonias con más de una central.

Entrada:
Un numero entero N que representa el número de colonias en la ciudad.
Matriz cuadrada de N x N que representa el grafo con las distancias en kilómetros entre las colonias de la ciudad.
Matriz cuadrada de N x N que representa las capacidades máximas de flujo de datos entre colonia i y colonia j.
Lista de N pares ordenados de la forma (A, B) que representan la ubicación en un plano coordenado de las centrales.

Salida:
1. Forma de cablear las colonias con fibra (lista de arcos de la forma (A, B)).
2. Ruta a seguir por el personal que reparte correspondencia, considerando inicio y fin en la misma colonia.
3. Valor de flujo máximo de información del nodo inicial al nodo final.
4. Lista de polígonos (cada elemento es una lista de puntos de la forma (x, y)).

Ejemplo de entrada:
4

0 16 45 32
16  0 18 21
45 18  0  7
32 21  7  0

0 48  12  18
52  0 42 32
18 46  0 56
24 36 52  0

(200, 500)
(300, 100)
(450, 150)
(520, 480)
*/

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <queue>
#include <limits>

#include "Arista.hpp"
#include "Coordenada.hpp"
#include "UnionBusqueda.hpp"

using namespace std;

// Función para encontrar el árbol de expansión mínima (MST) usando Kruskal
// Complejidad computacional: O(E log V) donde E es el número de aristas y V es el número de vértices
vector<Arista> kruskalMST(const vector<vector<int>>& matriz, int n) {
    vector<Arista> aristas;

    // Creación de todas las aristas posibles
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            if(i < j && matriz[i][j] > 0) {
                aristas.push_back({i, j, matriz[i][j]});
            }
        }
    }

    // Ordenar las aristas por peso
    sort(aristas.begin(), aristas.end(), [](const Arista& a, const Arista& b) {
        return a.peso < b.peso;
    });

    UnionBusqueda ub(n);
    vector<Arista> arbolExpansionMinima;

    // Construcción del MST
    for(const Arista& arista : aristas) {
        if(ub.buscar(arista.origen) != ub.buscar(arista.destino)) {
            ub.unir(arista.origen, arista.destino);
            arbolExpansionMinima.push_back(arista);
        }
    }
    return arbolExpansionMinima;
}

// Función para resolver el problema del Traveling Salesman Problem (TSP) por fuerza bruta
// Complejidad computacional: O(n!) donde n es el número de nodos
pair<int, vector<int>> tspFuerzaBruta(int nodoInicial, const vector<vector<int>>& matrizDistancia) {
    int N = matrizDistancia.size();
    vector<int> vertices;

    // Preparación inicial de vértices
    for (int i = 0; i < N; i++)
        if (i != nodoInicial)
            vertices.push_back(i);

    int pesoMinimo = INT_MAX;
    vector<int> rutaMinima;

    // Bucle para encontrar la ruta más corta
    do {
        int pesoCamino = 0, k = nodoInicial;
        vector<int> rutaActual = {nodoInicial};

        for (int i : vertices) {
            pesoCamino += matrizDistancia[k][i];
            k = i;
            rutaActual.push_back(i);
        }
        pesoCamino += matrizDistancia[k][nodoInicial];
        rutaActual.push_back(nodoInicial);

        if (pesoCamino < pesoMinimo) {
            pesoMinimo = pesoCamino;
            rutaMinima = rutaActual;
        }
    } while (next_permutation(vertices.begin(), vertices.end()));

    return {pesoMinimo, rutaMinima};
}

// Implementación de BFS para el algoritmo de Ford-Fulkerson
// Complejidad computacional: O(V + E) donde V es el número de vértices y E es el número de aristas
bool bfs(vector<vector<int>>& grafoResidual, int s, int t, vector<int>& padre) {
    vector<bool> visitado(grafoResidual.size(), false);
    queue<int> cola;
    cola.push(s);
    visitado[s] = true;
    padre[s] = -1;

    // Bucle principal de BFS
    while (!cola.empty()) {
        int u = cola.front();
        cola.pop();

        for (int v = 0; v < grafoResidual.size(); v++) {
            if (!visitado[v] && grafoResidual[u][v] > 0) {
                if (v == t) {
                    padre[v] = u;
                    return true;
                }
                cola.push(v);
                padre[v] = u;
                visitado[v] = true;
            }
        }
    }
    return false;
}

// Implementación del algoritmo de Ford-Fulkerson para encontrar el flujo máximo
// Complejidad computacional: O(E * f) donde E es el número de aristas y f es el flujo máximo
int fordFulkerson(vector<vector<int>>& matrizFlujoCapacidad, int s, int t) {
    int u, v;
    int N = matrizFlujoCapacidad.size();
    vector<vector<int>> grafoResidual(N, vector<int>(N));

    // Copiar la matriz de flujo de capacidad al grafo residual
    for (u = 0; u < N; u++)
        for (v = 0; v < N; v++)
             grafoResidual[u][v] = matrizFlujoCapacidad[u][v];

    vector<int> padre(N);
    int flujoMaximo = 0;

    // Bucle mientras exista un camino de aumento
    while (bfs(grafoResidual, s, t, padre)) {
        int flujoCamino = INT_MAX;

        for (v = t; v != s; v = padre[v]) {
            u = padre[v];
            flujoCamino = min(flujoCamino, grafoResidual[u][v]);
        }

        for (v = t; v != s; v = padre[v]) {
            u = padre[v];
            grafoResidual[u][v] -= flujoCamino;
            grafoResidual[v][u] += flujoCamino;
        }

        flujoMaximo += flujoCamino;
    }

    return flujoMaximo;
}

// Función para encontrar la central más cercana usando búsqueda lineal
// Complejidad computacional: O(n) donde n es el número de centrales
Coordenada centralMasCercana(const Coordenada& ubicacion, const vector<Coordenada>& centrales) {
    Coordenada centralCercana;
    int distanciaMinima = INT_MAX;

    // Bucle para encontrar la central más cercana
    for (const Coordenada& central : centrales) {
        int distancia = pow(central.x - ubicacion.x, 2) + pow(central.y - ubicacion.y, 2);
        if (distancia < distanciaMinima) {
            distanciaMinima = distancia;
            centralCercana = central;
        }
    }
    return centralCercana;
}

// Función para leer una matriz cuadrada
// Complejidad computacional: O(N^2) donde N es el número de filas y columnas de la matriz
void leerMatriz(vector<vector<int>>& matriz, int N, const string& descripcion) {
    cout << "\nIngrese la matriz cuadrada que representa " << descripcion << ":\n";
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            cin >> matriz[i][j];
        }
    }
}

// Función para imprimir el árbol de expansión mínima (MST)
// Complejidad computacional: O(N) donde N es el número de aristas del MST
void imprimirMST(const vector<Arista>& mst) {
    cout << "\nParte 1. Forma óptima de cablear las colonias con fibra óptica:\n";
    for (const Arista& a : mst) {
        cout << "(" << char('A' + a.origen) << ", " << char('A' + a.destino) << ")" << endl;
    }
}

// Función para imprimir la ruta del Traveling Salesman Problem (TSP)
// Complejidad computacional: O(N) donde N es el número de nodos del grafo
void imprimirRutaTSP(const vector<int>& ruta) {
    cout << "\nParte 2. Ruta a seguir por el personal que reparte correspondencia:\n";
    for (size_t i = 0; i < ruta.size(); i++) {
        cout << char('A' + ruta[i]);
        if (i < ruta.size() - 1) cout << " --> ";
    }
    cout << endl;
}

// Función principal
int main() {
    int N;
    cout << "Ingrese el número de colonias en la ciudad: ";
    cin >> N;

    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    vector<vector<int>> matrizDistancia(N, vector<int>(N));
    leerMatriz(matrizDistancia, N, "las distancias en kilómetros entre las colonias");

    vector<vector<int>> matrizFlujoCapacidad(N, vector<int>(N));
    leerMatriz(matrizFlujoCapacidad, N, "las capacidades máximas de flujo de datos entre colonias");

    cout << "\nIngrese la lista de pares ordenados de la forma (x, y) que representan la ubicación de las centrales:\n";
    vector<Coordenada> ubicacionesCentrales(N);
    char temp;
    for (int i = 0; i < N; ++i) {
        cin >> temp >> ubicacionesCentrales[i].x >> temp >> ubicacionesCentrales[i].y >> temp;
    }

    Coordenada nuevaContratacion;
    cout << "\nIngrese las coordenadas de la nueva contratación de la forma (x, y): ";
    cin >> temp >> nuevaContratacion.x >> temp >> nuevaContratacion.y >> temp;

    vector<Arista> mst = kruskalMST(matrizDistancia, N);
    imprimirMST(mst);

    auto resultadoTSP = tspFuerzaBruta(0, matrizDistancia);
    imprimirRutaTSP(resultadoTSP.second);

    int flujoMaximo = fordFulkerson(matrizFlujoCapacidad, 0, N-1);
    cout << "\nParte 3. Valor de flujo máximo de información del nodo inicial al nodo final:\n" << flujoMaximo << endl;

    Coordenada centralCercana = centralMasCercana(nuevaContratacion, ubicacionesCentrales);
    cout << "\nParte 4. Coordenadas de la central más cercana para la nueva contratación:\n(" << centralCercana.x << ", " << centralCercana.y << ")" << endl;

    return 0;
}

/*
Caso de prueba:
Ingrese el número de colonias en la ciudad: 4

Ingrese la matriz cuadrada que representa las distancias en kilómetros entre las colonias:
 0 16 45 32
16  0 18 21
45 18  0  7
32 21  7  0

Ingrese la matriz cuadrada que representa las capacidades máximas de flujo de datos entre colonias:
 0 48  12  18
52  0 42 32
18 46  0 56
24 36 52  0

Ingrese la lista de pares ordenados de la forma (x, y) que representan la ubicación de las centrales:
(200,500)
(300,100)
(450,150)
(520,480)

Ingrese las coordenadas de la nueva contratación de la forma (x, y): (400, 300)

Parte 1. Forma óptima de cablear las colonias con fibra óptica:
(C, D)
(A, B)
(B, C)

Parte 2. Ruta a seguir por el personal que reparte correspondencia:
A --> B --> C --> D --> A

Parte 3. Valor de flujo máximo de información del nodo inicial al nodo final:
78

Parte 4. Coordenadas de la central más cercana para la nueva contratación:
(450, 150)
*/
