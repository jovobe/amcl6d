/*
 * Face.cpp
 *
 *  Created on: 10.08.2014
 *      Author: Sebastian Höffner
 */

#include "Face.h"

Face::Face() 
{
    vertex_vec = std::vector<Vertex*>();
}

Face::~Face()
{
    
}

void Face::add_vertex(Vertex* vertex)
{
    vertex_vec.push_back(vertex);
}

Vertex* Face::get_vertex(int idx)
{
    return vertex_vec.at(idx);
}

int Face::vertex_count()
{
    return vertex_vec.size();
}
