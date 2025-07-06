#include <map>
#include <fstream>
#include <sstream>
#include <numbers>

#include "Utils.hpp"
#include "Topology.hpp"
#include "Polyhedron.hpp"


using namespace std;
using namespace Eigen;



Polyhedron build_platonic_solid(int p, int q)
{
	Polyhedron solid;
	
	// Gestione dei casi noti
	if(p == 3 && q == 3)
	{	solid.id = 1;
		
		double a = sqrt(3)/3;
		solid.vertices = {
			{0, Vector3d(a,a,a)}, {1, Vector3d(a,-a,-a)}, 
			{2, Vector3d(-a,a,-a)}, {3, Vector3d(-a,-a,a)}
		};

		solid.faces = {
			{0, {0,1,2}},
			{1, {0,3,1}},
			{2, {0,2,3}},
			{3, {1,3,2}}
		};
		cout << " Poliedro di partenza: Tetraedro ( p = 3 e q = 3 )" << endl << endl;
	}    

	else if ((p == 3 && q == 4) || (p == 4 && q == 3 )) // cubo e ottaedro 
		{   solid.id = 2;
			solid.vertices = {
				{0, Vector3d(1,0,0)}, {1, Vector3d(-1,0,0)}, 
				{2, Vector3d(0,1,0)}, {3, Vector3d(0,-1,0)},
				{4, Vector3d(0,0,1)}, {5, Vector3d(0,0,-1)}
			};

			solid.faces = {
				{0, {0,2,4}}, {1, {2,1,4}}, {2, {1,3,4}}, {3, {3,0,4}},
				{4, {2,0,5}}, {5, {1,2,5}}, {6, {3,1,5}}, {7, {0,3,5}}
			};

			if (p == 3){ cout << " Poliedro di partenza: Ottaedro ( p = 3 e q = 4 )" << endl << endl; }
			else { cout << " Poliedro di partenza: Cubo ( p = 4 e q = 3 )" << endl << endl; }
		}

	else if ((p == 3 && q == 5) || (p == 5 && q == 3)) //dodecaedro e icosaedro
		{   solid.id = 3;
			const double phi = (1 + sqrt(5))/2;
			double norm = sqrt(1 + phi*phi);

			vector<Vector3d> positions = {
				{0, 1, phi}, {0, -1, phi}, {0, 1, -phi}, {0, -1, -phi},
				{1, phi, 0}, {-1, phi, 0}, {1, -phi, 0}, {-1, -phi, 0},
				{phi, 0, 1}, {-phi, 0, 1}, {phi, 0, -1}, {-phi, 0, -1}
			};

			for(size_t i=0; i<positions.size(); i++)
				solid.vertices.push_back({static_cast<unsigned int>(i), positions[i]/norm});

			solid.faces = {
				{0,{0,1,8}}, {1,{0,8,4}}, {2,{0,4,5}}, {3,{0,5,9}}, {4,{0,9,1}},
				{5,{1,6,8}}, {6,{1,9,7}}, {7,{1,7,6}}, {8,{8,6,10}}, {9,{8,10,4}},
				{10,{4,10,2}}, {11,{4,2,5}}, {12,{5,2,11}}, {13,{5,11,9}},
				{14,{9,11,7}}, {15,{7,11,3}}, {16,{7,3,6}}, {17,{6,3,10}},
				{18,{10,3,2}}, {19,{2,3,11}}
			};

			if (p == 3){ cout << " Poliedro di partenza: Icosaedro ( p = 3 e q = 5 )" << endl << endl; }
			else { cout << " Poliedro di partenza: Dodecaedro ( p = 5 e q = 3 )" << endl << endl; }
		}

		else {
			throw std::runtime_error(" Solido non supportato. ");
	}

	// COSTRUZIONE PRECISA E CHIARA DEI LATI (SPIGOLI):
	map<pair<unsigned int,unsigned int>, unsigned int> edgeIndexMap;
	unsigned int edge_id = 0;

	for (auto& face : solid.faces)
	{
		face.id_edges.clear();

		for (unsigned int j=0; j<face.id_vertices.size(); ++j)
		{
			unsigned int v_start = face.id_vertices[j];
			unsigned int v_end = face.id_vertices[(j+1)%3];

			pair<unsigned int,unsigned int> current_edge = minmax(v_start, v_end);

			// Se il lato non esiste, lo creo
			if(edgeIndexMap.find(current_edge) == edgeIndexMap.end())
			{
				solid.edges.push_back({edge_id, current_edge.first, current_edge.second});
				edgeIndexMap[current_edge] = edge_id;
				face.id_edges.push_back(edge_id);
				edge_id++;
			}
			else
			{
				face.id_edges.push_back(edgeIndexMap[current_edge]);
			}
		}
	}

	// Restituisci il poliedro completo di vertici, spigoli e facce
	return solid;
}



// Esporta il poliedro in formato compatibile con Paraview
void export_polyhedron(const Polyhedron& P)
{   

	// Matrice coordinate vertici
	MatrixXd coordsCell0D = MatrixXd::Zero(3, P.n_vertices());
	for (const auto& v : P.vertices)
		coordsCell0D.col(v.id) = v.coords;

	// Matrice estremi degli edge
	MatrixXi extremaCell1D = MatrixXi::Zero(2, P.n_edges());
	for (unsigned int i = 0; i < P.edges.size(); ++i) {
		extremaCell1D(0, i) = P.edges[i].origin;
		extremaCell1D(1, i) = P.edges[i].end;
	}

	// Evidenzia il cammino sul poliedro

	// Vettore per evidenziare i vertici sul cammino
	vector<double> visitedNodes(P.n_vertices(), 0.0);
	for (unsigned int i = 0; i < P.n_vertices(); ++i)
		visitedNodes[i] = P.vertices[i].short_path ? 1.0 : 0.0;

	// Inizializza la struttura UCDProperty (nodes)
	Gedim::UCDProperty<double> visitedNodes_UCD;
	visitedNodes_UCD.NumComponents = 1;

	// Imposta gli attributi
	const double* ptr1 = visitedNodes.data();
	visitedNodes_UCD.Data = ptr1;
	visitedNodes_UCD.Label = "Nodi Visitati";

	// Inizializza il vettore di strutture UCDProperty 
	vector<Gedim::UCDProperty<double>> points_properties = { visitedNodes_UCD };


	// Vettore per evidenziare gli edge sul cammino
	vector<double> visitedEdges(P.n_edges(), 0.0);
	for (unsigned int i = 0; i < P.n_edges(); ++i)
		visitedEdges[i] = P.edges[i].short_path ? 1.0 : 0.0;



	// Inizializza la struttura UCDProperty (edges)
	Gedim::UCDProperty<double> visitedEdges_UCD;
	visitedEdges_UCD.NumComponents = 1;

	// Imposta gli attributi
	const double* ptr2 = visitedEdges.data();
	visitedEdges_UCD.Data = ptr2;
	visitedEdges_UCD.Label = "ShortPath";
	
	// Inizializza il vettore di strutture UCDProperty per gli edge
	vector<Gedim::UCDProperty<double>> segments_properties = { visitedEdges_UCD };


	// Crea la struttura UCDUtilities
	Gedim::UCDUtilities utilities;

	// Chiama le funzioni di esportazione
	utilities.ExportPoints("./Cell0Ds.inp",
							coordsCell0D,
							points_properties);

	utilities.ExportSegments("./Cell1Ds.inp",
							 coordsCell0D,
							 extremaCell1D,
							 points_properties,
							 segments_properties);
}


// Scrive i file di output (txt) con vertici, spigoli, facce e solido
bool write_output(const Polyhedron& P)
{
	// Esporta i vertici (0D)
	ofstream ofs_cell0D("Cell0Ds.txt");
	if (!ofs_cell0D) {
		cerr << "Errore nella creazione del file Cell0Ds.txt ." << endl;
		return false;
	}
	ofs_cell0D << "id;X;Y;Z" << endl;
	for (const auto& v : P.vertices)
		ofs_cell0D << v.id << ";" << v.coords[0] << ";" << v.coords[1] << ";" << v.coords[2] << endl;
	ofs_cell0D.close();


	// Esporta gli edge (1D)
	ofstream ofs_cell1D("Cell1Ds.txt");
	if (!ofs_cell1D) {
		cerr << "Errore nella creazione del file Cell1Ds.txt ." << endl;
		return false;
	}
	ofs_cell1D << "id;origin;end" << endl;
	for (const auto& e : P.edges)
		ofs_cell1D << e.id << ";" << e.origin << ";" << e.end << endl;
	ofs_cell1D.close();


	// Esporta le facce (2D)
	ofstream ofs_cell2D("Cell2Ds.txt");
	if (!ofs_cell2D) {
		cerr << "Errore nella creazione del file Cell2Ds.txt ." << endl;
		return false;
	}
	ofs_cell2D << "id;NumVertices;Vertices;NumEdges;Edges" << endl;
	for (const auto& f : P.faces) {
		ofs_cell2D << f.id << ";" << f.n_vertices() << ";";
		for (unsigned int i = 0; i < f.n_vertices(); ++i)
			ofs_cell2D << f.id_vertices[i] << ";";
		ofs_cell2D << f.n_edges() << ";";
		for (unsigned int i = 0; i < f.n_edges(); ++i) {
			ofs_cell2D << f.id_edges[i];
			if (i < f.n_edges() - 1) ofs_cell2D << ";";
		}
		ofs_cell2D << endl;
	}
	ofs_cell2D.close();


	// Esporta il solido (3D)
	ofstream ofs_cell3D("Cell3Ds.txt");
	if (!ofs_cell3D) {
		cerr << "Errore nella creazione del file Cell3Ds.txt ." << endl;
		return false;
	}
	ofs_cell3D << "id;NumVertices;Vertices;NumEdges;Edges;NumFaces;Faces" << endl;
	ofs_cell3D << P.id << ";" << P.n_vertices() << ";";
	for (unsigned int i = 0; i < P.n_vertices(); ++i)
		ofs_cell3D << P.vertices[i].id << ";";
	ofs_cell3D << P.n_edges() << ";";
	for (unsigned int i = 0; i < P.n_edges(); ++i)
		ofs_cell3D << P.edges[i].id << ";";
	ofs_cell3D << P.n_faces() << ";";
	for (unsigned int i = 0; i < P.n_faces(); ++i) {
		ofs_cell3D << P.faces[i].id;
		if (i < P.n_faces() - 1) ofs_cell3D << ";";
	}
	ofs_cell3D << endl;
	ofs_cell3D.close();
	return true;

}