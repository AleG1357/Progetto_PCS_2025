#include "Utils.hpp"
#include "Triangulation.hpp"
#include "Topology.hpp"
#include "ShortPath.hpp"


using namespace std;


int main(int argc, char* argv[]) {

	cout << endl;
	 
	if (argc != 5 && argc != 7) {
		cerr << "Utilizzo corretto: " << argv[0] << " <p> <q> <b> <c> <id_start> <id_end>" << endl;
		return 1;
	}

	int tmp_p = std::stoi(argv[1]);
	int tmp_q = std::stoi(argv[2]);
	int tmp_b = std::stoi(argv[3]);
	int tmp_c = std::stoi(argv[4]);

	if (tmp_p < 0 || tmp_q < 0 || tmp_b < 0 || tmp_c < 0) {
		cerr << " Errore: tutti i parametri devono essere >= 0!" << std::endl;
		return 1;
	}

	unsigned int p = static_cast<unsigned int>(tmp_p);
	unsigned int q = static_cast<unsigned int>(tmp_q);
	unsigned int b = static_cast<unsigned int>(tmp_b);
	unsigned int c = static_cast<unsigned int>(tmp_c);

	// Determina classe e parametro di suddivisione
	unsigned int delta = 0; // Parametro di triangolazione
	unsigned int classe = 0; // 1=Classe I, 2=Classe II
	
	//Creo il polidero P;
	Polyhedron P;

	try {
		P = build_platonic_solid(p, q);
	} catch (const std::exception& ex) {
		cerr << " Errore di input: p e q non validi \n Utilizzo corretto: [ (p,q) ∈ {3,4,5} ] e [ p = 3 o q = 3 ] " << ex.what() << endl;
		return 1;
	}

	
	//Triangolazione del poliedro
	if((b >= 1 && c == 0) || (b == 0 && c >=1)){
		classe = 1; //classe 1
		delta = max(b,c);
		cout << " Triangolazione d Classe I di parametro: " << delta << endl << endl;
		P = triangulate_classI(P, delta);
	}
	else if (b == c){
		classe = 2;
		delta = b;
		cout << " Triangolazione di Classe II di parametro: " << delta << endl << endl;
		P = triangulate_classII(P, delta);
	} else {
		cerr << " b e c non validi" << endl;
		return 1;
	}


	
	// Proiezione dei vertici sulla sfera unitaria
	for(auto& v : P.vertices)
	{
		project_on_unit_sphere(v);
	}
	
	
	// Prendo le adiacenze dei vertici e degli spigoli
	assign_edge_adjacencies(P);
	assign_vertex_adjacencies(P);


	// Se p diverso da 3, costruisco il poliedro duale
	if(p != 3)
	{
		P = dual_polyhedron(P);
		cout << " Poliedro di Goldberg " << endl << endl;
		// Get new neighbors
		assign_edge_adjacencies(P);
		assign_vertex_adjacencies(P);
	}
	else { cout << " Poliedro Geodetico " << endl << endl;}

	// Shortest path 
	if(argc == 7)
	{
		unsigned int id_path_start = std::stoi(argv[5]);
		unsigned int id_path_end = std::stoi(argv[6]);
		if (id_path_start >= P.n_vertices() || id_path_end >= P.n_vertices()) {
			cerr << " Input invalido: ID dei vertici non appartenenti al poliedro " << endl;
			return 1;
		}
		if (id_path_start == id_path_end) {
			cerr << " id_start = id_end ---> Distanza = 0 " << endl;
			return 1;
		}

		// Costruisci grafo e matrice pesi
		Graph graph = build_adjacency_graph(P);
		MatrixXd weights = build_edge_weight_matrix(P);

		// Scegli algoritmo in base alla classe (classe==1: BFS, classe==2: Dijkstra)
		bool isUniformEdgeLength = (classe == 1);
		vector<unsigned int> path = find_shortest_path(graph, id_path_start, id_path_end, isUniformEdgeLength, weights);
       
		cout << " Il cammino più breve tra i vertici " << id_path_start << " e " << id_path_end << " passa per i vertici: ";
		// Evidenzia e stampa il percorso
		highlight_path(P, path);
		display_path(P, path);
		//cout << " Percorso trovato tra i vertici " << id_path_start << " e " << id_path_end << endl;
	}


	// Export the polyhedron for Paraview
	export_polyhedron(P);
		
	// Write .txt output files
	if(!write_output(P))
	{
		return 1;
	}
	
	cout << endl;

	return 0;
	
}