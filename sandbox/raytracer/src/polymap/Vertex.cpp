/*
 * Vertex.cpp
 *
 *  Created on: 10.08.2014
 *      Author: Sebastian Höffner
 */

#include "Vertex.h"

std::vector<std::string> Vertex::s_prop_vec = std::vector<std::string>();

Vertex::Vertex() 
{
    m_properties = std::map<std::string, double>();
}

Vertex::~Vertex()
{
}

void Vertex::add_property(std::string property)
{
    s_prop_vec.push_back(property);
}

std::string Vertex::get_property(int index)
{
    return s_prop_vec[index];
}

int Vertex::property_count()
{
    return s_prop_vec.size();
}

void Vertex::print_property_list()
{
    std::cout << "prop list: " << std::endl;
    for(int i = 0; i < s_prop_vec.size(); i++) 
    {
        std::cout << s_prop_vec[i] << std::endl;
    }
    std::cout << "end prop" << std::endl;
}

void Vertex::set_value(std::string property, double value)
{
    m_properties.insert(std::pair<std::string, double>(property, value));
}

double Vertex::get_value(std::string property)
{
    return m_properties[property];
}

std::ostream& operator<<(std::ostream& os, Vertex& v)
{
    os << "Vertex: " << std::endl;
    for(int i = 0; i < Vertex::property_count(); i++) 
    {
        os << "  " << Vertex::get_property(i) 
           << "="  << v.get_value(Vertex::get_property(i)) << std::endl;
    }
    os << "End Vertex";
    return os;
}