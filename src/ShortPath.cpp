#include "ShortPath.hpp"


using namespace std;
using namespace Eigen;


// Crea la lista di adiacenza dei vertici a partire dagli spigoli del poliedro
// Ogni vertice ha la lista degli ID dei vertici adiacenti
Graph build_adjacency_graph(const Polyhedron& poly) {
    Graph g;
    g.adjacency_list.resize(poly.n_vertices());
    for (const auto& edge : poly.edges) {
        unsigned int u = edge.origin;
        unsigned int v = edge.end;
        if (std::find(g.adjacency_list[u].begin(), g.adjacency_list[u].end(), v) == g.adjacency_list[u].end())
            g.adjacency_list[u].push_back(v);
        if (std::find(g.adjacency_list[v].begin(), g.adjacency_list[v].end(), u) == g.adjacency_list[v].end())
            g.adjacency_list[v].push_back(u);
    }
    return g;
}

// Crea la matrice dei pesi: W(i,j) = distanza euclidea tra i vertici i e j se adiacenti, altrimenti infinito
MatrixXd build_edge_weight_matrix(const Polyhedron& poly) {
    MatrixXd W = MatrixXd::Constant(poly.n_vertices(), poly.n_vertices(), std::numeric_limits<double>::max());
    for (unsigned int i = 0; i < poly.n_vertices(); ++i) W(i, i) = 0.0;
    for (const auto& edge : poly.edges) {
        double len = (poly.vertices[edge.origin].coords - poly.vertices[edge.end].coords).norm();
        W(edge.origin, edge.end) = len;
        W(edge.end, edge.origin) = len;
    }
    return W;
}




// Algoritmo di Dijkstra: trova il cammino minimo tra due vertici in un grafo pesato
vector<unsigned int> dijkstra_alg(const Graph& g, unsigned int start, unsigned int end, const MatrixXd& W) {
    unsigned int N = g.adjacency_list.size();
    vector<double> dist(N, std::numeric_limits<double>::max());
    vector<int> pred(N, -1);
    dist[start] = 0.0;
    pred[start] = start;
    using QElem = pair<double, unsigned int>;
    priority_queue<QElem, vector<QElem>, greater<QElem>> pq;
    pq.push({0.0, start});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (u == end) break;
        if (d > dist[u]) continue;
        for (auto v : g.adjacency_list[u]) {
            double alt = dist[u] + W(u, v);
            if (alt < dist[v]) {
                dist[v] = alt;
                pred[v] = u;
                pq.push({alt, v});
            }
        }
    }
    vector<unsigned int> path;
    if (dist[end] == std::numeric_limits<double>::max()) return path;
    for (unsigned int v = end; v != start; v = pred[v]) path.push_back(v);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// Algoritmo BFS: trova il cammino minimo tra due vertici in un grafo non pesato
vector<unsigned int> bfs_alg(const Graph& g, unsigned int start, unsigned int end) {
    unsigned int N = g.adjacency_list.size();
    vector<bool> visited(N, false);
    vector<int> pred(N, -1);
    queue<unsigned int> q;
    visited[start] = true;
    pred[start] = start;
    q.push(start);
    while (!q.empty()) {
        unsigned int u = q.front(); q.pop();
        if (u == end) break;
        for (auto v : g.adjacency_list[u]) {
            if (!visited[v]) {
                visited[v] = true;
                pred[v] = u;
                q.push(v);
            }
        }
    }
    vector<unsigned int> path;
    if (!visited[end]) return path;
    for (unsigned int v = end; v != start; v = pred[v]) path.push_back(v);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// Sceglie automaticamente tra BFS e Dijkstra in base alla tipologia del grafo (spigoli uniformi o pesati)
vector<unsigned int> find_shortest_path(const Graph& g, unsigned int start, unsigned int end, bool is_uniform_edge_length, const MatrixXd& W) {
    if (is_uniform_edge_length) {
        return bfs_alg(g, start, end);
    } else {
        return dijkstra_alg(g, start, end, W);
    }
}


// Stampa il cammino minimo trovato e la sua lunghezza
void display_path(const Polyhedron& poly, const vector<unsigned int>& path) {
    for (size_t i = 0; i < path.size(); ++i) {
    cout << path[i];
    if (i != path.size() - 1) cout << " → ";
}
cout << endl << endl;
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        auto& u = poly.vertices[path[i-1]].coords;
        auto& v = poly.vertices[path[i]].coords;
        total += (u - v).norm();
    }
    cout << " Numero di lati attraversati: " << (path.size() > 1 ? path.size() - 1 : 0) << endl << endl;
    cout << " Lunghezza totale del cammino: " << total << endl;
}


// Evidenzia i vertici e gli spigoli del cammino minimo nel poliedro (per esportazione/visualizzazione)
void highlight_path(Polyhedron& poly, const vector<unsigned int>& path) {
    for (auto idx : path) poly.vertices[idx].short_path = true;
    for (size_t i = 1; i < path.size(); ++i) {
        unsigned int u = path[i-1], v = path[i];
        for (auto& e : poly.edges) {
            if ((e.origin == u && e.end == v) || (e.origin == v && e.end == u)) e.short_path = true;
        }
    }
}