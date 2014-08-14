/*
 * Face.h
 *
 *  Created on: 10.08.2014
 *  Author: Sebastian Höffner
 */

#ifndef FACE_H
#define FACE_H

#include <string>
#include <iostream>
#include "../Logger.h"
#include "Vertex.h"

class Face
{
    
public:
    /**
	 * Default Constructor
	 */
	Face();
    
    /**
     * Destructor
     */
	~Face();

    /**
     * adds a vertex to this face
     * @param vertex the vertex to be added
     */
    void add_vertex(Vertex* vertex);
    
    /**
     * returns a handle to the idx-th vertex
     */
    Vertex* get_vertex(int idx);
    
    /**
     * returns the number of vertices in this face
     */
    int vertex_count();

private:

    std::vector<Vertex*> vertex_vec;
};
#endif /* FACE_H */