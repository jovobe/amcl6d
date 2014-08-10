/*
 * PolyMap.h
 *
 *  Created on: 10.08.2014
 *  Author: Sebastian Höffner
 */

#ifndef POLYMAP_H_
#define POLYMAP_H_

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "../Logger.h"
#include "Vertex.h"
#include "Face.h"

class PolyMap
{
    
public:
    /**
	 * Default Constructor
     * @param filename ply-file
	 */
	PolyMap(std::string filename);

	/**
	 * Destructor
	 */
	~PolyMap();

    /**
     * Returns the number of faces in the mesh.
     */
    int face_count();
    
    /**
     * Returns the face with index index
     */
    Face get_face(int index);
    

private:
	/**
	 * Loads the given ply file and parses it
	 * @param filename a valid ply file (ascii 1.0)
	 */
	void load_file(std::string filename);
    
    int m_facecount;
    int m_vertexcount;
    
    std::vector<Vertex> m_vertices;
    std::vector<Face>   m_faces;
    
    std::string m_filename;
    
    std::string ltrim(std::string str);
    std::string rtrim(std::string str);
    std::string trim(std::string str);

};

#endif /* POLYMAP_H_ */
