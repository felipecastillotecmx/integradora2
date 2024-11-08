#pragma once

#include <vector>
using namespace std;

// Estructura para el manejo de la unión-búsqueda (para Kruskal)
struct UnionBusqueda {
    vector<int> padre;

    // Constructor: Inicializa cada nodo como su propio padre
    UnionBusqueda(int n) : padre(n) {
        for(int i = 0; i < n; i++) padre[i] = i;
    }

    // Encuentra la raíz del conjunto al que pertenece x
    int buscar(int x) {
        if(x != padre[x]) padre[x] = buscar(padre[x]);
        return padre[x];
    }

    // Une dos conjuntos
    void unir(int x, int y) {
        padre[buscar(x)] = buscar(y);
    }
};
