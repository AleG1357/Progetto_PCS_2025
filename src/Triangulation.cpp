#include "Triangulation.hpp"
#include "Topology.hpp"


using namespace std;
using namespace Eigen;


// Aggiunge un vertice solo se non già presente nel poliedro (con tolleranza sulle coordinate)
unsigned int complete_vertex(Polyhedron& poly, const Vector3d& coords)
{
	constexpr double tol = 1e-12;
	for (const auto& v : poly.vertices)
	{
		if ((v.coords - coords).norm() < tol)
			return v.id;
	}
	Vertex v_new;
	v_new.coords = coords;
	v_new.id = static_cast<unsigned int>(poly.vertices.size());
	poly.vertices.push_back(v_new);
	return v_new.id;
}

// Aggiunge uno spigolo tra due vertici solo se non già presente (può forzare l'ID)
unsigned int complete_edge(Polyhedron& poly, unsigned int id1, unsigned int id2, unsigned int forced_id)
{
	if (id1 >= poly.vertices.size() || id2 >= poly.vertices.size() || id1 == id2)
	{
		cerr << " Edge non valido" << endl;
		return numeric_limits<unsigned int>::max();
	}
	for (const auto& e : poly.edges)
	{
		if ((e.origin == id1 && e.end == id2) || (e.origin == id2 && e.end == id1))
			return e.id;
	}
	Edge e_new;
	e_new.origin = id1;
	e_new.end = id2;
	if (forced_id != numeric_limits<unsigned int>::max())
	{
		for (const auto& e : poly.edges)
			if (e.id == forced_id)
			{
				cerr << " ID edge già usato" << endl;
				return numeric_limits<unsigned int>::max();
			}
		e_new.id = forced_id;
	}
	else
	{
		e_new.id = static_cast<unsigned int>(poly.edges.size());
	}
	poly.edges.push_back(e_new);
	return e_new.id;
}


// Triangolazione Classe I: suddivide ogni faccia in delta^2 triangoli
Polyhedron triangulate_classI(const Polyhedron& P_old, const unsigned int& delta)
{
	Polyhedron P;
	P.id = P_old.id;
	if (delta == 1) return P_old;
	unsigned int T = delta * delta;
	switch (P_old.id) {
		case 1: // Tetraedro
			P.vertices.reserve(2 * T + 2);
			P.edges.reserve(6 * T);
			P.faces.reserve(4 * T);
			break;
		case 2: // Ottaedro
			P.vertices.reserve(4 * T + 2);
			P.edges.reserve(12 * T);
			P.faces.reserve(8 * T);
			break;
		default: // Icosaedro o altro
			P.vertices.reserve(10 * T + 2);
			P.edges.reserve(30 * T);
			P.faces.reserve(20 * T);
	}
	for (const auto& face : P_old.faces)
	{
		unsigned int idA = face.id_vertices[0];
		unsigned int idB = face.id_vertices[1];
		unsigned int idC = face.id_vertices[2];
		const auto& A = P_old.vertices[idA];
		const auto& B = P_old.vertices[idB];
		const auto& C = P_old.vertices[idC];
		map<pair<unsigned int, unsigned int>, unsigned int> Vij;
		for (unsigned int i = 0; i <= delta; ++i)
			for (unsigned int j = 0; j <= delta - i; ++j)
			{
				Vector3d coords = (i * A.coords + j * B.coords + (delta - i - j) * C.coords) / delta;
				unsigned int id_v = complete_vertex(P, coords);
				Vij[{i, j}] = id_v;
			}
		for (unsigned int i = 0; i < delta; ++i)
			for (unsigned int j = 0; j < delta - i; ++j)
			{
				unsigned int id0 = Vij[{i, j}];
				unsigned int id1 = Vij[{i + 1, j}];
				unsigned int id2 = Vij[{i, j + 1}];
				unsigned int e0 = complete_edge(P, id0, id1);
				unsigned int e1 = complete_edge(P, id1, id2);
				unsigned int e2 = complete_edge(P, id2, id0);
				Face f0;
				f0.id = static_cast<unsigned int>(P.faces.size());
				f0.id_vertices = {id0, id1, id2};
				f0.id_edges = {e0, e1, e2};
				P.faces.push_back(f0);
				if (!P.check_faces()) {
					P.faces.pop_back();
					cerr << " Poliedro non valido" << std::endl;
				}
				if (i + j < delta - 1)
				{
					unsigned int id3 = Vij[{i + 1, j + 1}];
					unsigned int e3 = complete_edge(P, id1, id2);
					unsigned int e4 = complete_edge(P, id2, id3);
					unsigned int e5 = complete_edge(P, id3, id1);
					Face f1;
					f1.id = static_cast<unsigned int>(P.faces.size());
					f1.id_vertices = {id1, id2, id3};
					f1.id_edges = {e3, e4, e5};
					P.faces.push_back(f1);
					if (!P.check_faces()) {
						P.faces.pop_back();
						cerr << " Poliedro non valido" << std::endl;
					}
				}
			}
	}
	return P;
}
// Triangolazione Class II di un poliedro con parametro delta (b = c = delta > 0)
Polyhedron triangulate_classII(const Polyhedron& P_old, const unsigned int& delta)
{
	Polyhedron P;
	P.id = P_old.id;
	if (delta == 1) return P_old;
	Polyhedron P_I = triangulate_classI(P_old, delta);
	assign_edge_adjacencies(P_I);
	P.vertices = P_I.vertices;
	unordered_map<unsigned int, unsigned int> face_barycenter;
	for (const auto& f : P_I.faces) {
		Vertex bc = face_centroid(P_I, f.id);
		bc.id = P.n_vertices();
		face_barycenter[f.id] = bc.id;
		P.vertices.push_back(bc);
	}
	for (const auto& e : P_I.edges) {
		unsigned int v0 = e.origin, v1 = e.end;
		unsigned int bc0 = face_barycenter[e.face_adjacencies[0]];
		unsigned int bc1 = face_barycenter[e.face_adjacencies[1]];
		unsigned int e0 = complete_edge(P, v0, bc0);
		unsigned int e1 = complete_edge(P, bc0, bc1);
		unsigned int e2 = complete_edge(P, bc1, v0);
		Face f0;
		f0.id = P.faces.size();
		f0.id_vertices = {v0, bc0, bc1};
		f0.id_edges = {e0, e1, e2};
		P.faces.push_back(f0);
		unsigned int e3 = complete_edge(P, bc0, v1);
		unsigned int e4 = complete_edge(P, v1, bc1);
		Face f1;
		f1.id = P.faces.size();
		f1.id_vertices = {bc0, v1, bc1};
		f1.id_edges = {e3, e4, e1};
		P.faces.push_back(f1);
	}
	return P;
}