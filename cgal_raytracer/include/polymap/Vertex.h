/*
 * Vertex.h
 *
 *  Created on: 10.08.2014
 *  Author: Sebastian Höffner
 *
 * Be careful while using this, there is no error checking included.
 */

#ifndef VERTEX_H
#define VERTEX_H

#include <string>
#include <iostream>
#include <vector>
#include <map>
#include "../Logger.h"

class Vertex
{
    
public:
    /**
	 * Default Constructor
     * @param filename ply-file
	 */
	Vertex();
    
    /**
     * Destructor
     */
	~Vertex();

    /**
     * adds a property to the property list
     * @param property the property name
     */
    static void add_property(std::string property);
    
    /**
     * gets the property name for the index-th property.
     * @param index property index
     */
    static std::string get_property(int index);
    
    /**
     * returns the number of properties
     */
    static int property_count();
    
    /**
     * prints all properties to std::cout
     */
    static void print_property_list();
    
    /**
     * sets the value of a property
     * @param property the name of the property to be set
     * @value the value to set
     */
    void set_value(std::string property, double value);
    
    /**
     * gets the value of the given property
     * @param property name of the property
     */
    double get_value(std::string property);
    
    /**
     * pipe a vertex to a stream
     */
    friend std::ostream& operator<<(std::ostream& os, Vertex& v);
    
private:
    static std::vector<std::string> s_prop_vec;
    
    std::map<std::string, double> m_properties;
};

#endif /* VERTEX_H */