/*
 * PolyMap.cpp
 *
 *  Created on: 10.08.2014
 *      Author: Sebastian Höffner
 *    Modified: 14.08.2014
 */

#include "PolyMap.h"

PolyMap::PolyMap(std::string filename) 
{
    m_facecount = 0;
    m_vertexcount = 0;
    m_faces = std::vector<Face>();
    m_vertices = std::vector<Vertex>();
    load_file(filename);
}

PolyMap::~PolyMap()
{
}

int PolyMap::face_count()
{
    return m_facecount;
}

int PolyMap::vertex_count()
{
    return m_vertexcount;
}

Face PolyMap::get_face(int index)
{
    if(index < 0 || index > m_facecount)
    {
        Logger::instance()->logX("is", index, "out of bounds in PolyMap::get_face(int)");
        // ugly workaround
        return m_faces.at(0);
    }
    return m_faces.at(index);
}
 
Vertex PolyMap::get_vertex(int index)
{
  if(index < 0 || index > m_vertexcount)
  {
    Logger::instance()->logX("is", index, "out of bounds in PolyMap::get_vertex(int)");
    return m_vertices.at(0);
  }
  return m_vertices.at(index);
}

void PolyMap::load_file(std::string fn)
{
    m_filename = fn;
    
    Logger::instance()->logX("ss", "Reading", m_filename.c_str());
    std::ifstream file(m_filename.c_str());
    if(file.good())
    {
        std::string line;
        
        // check for ply at the first line
        getline(file, line);
        if(line.compare("ply"))
        {
            Logger::instance()->log("File has to start with \"ply\"! Aborting.");
            file.close();
            return;
        }
        
        // parse header information
        while(getline(file, line) && line.compare("end_header"))
        {
            // trim whitespace
            line = trim(line);
            
            std::string identifier = line.substr(0, line.find_first_of(" "));
            // skip comments
            if(!identifier.compare("comment"))
            {
                continue;
            }
            
            // check format
            if(!identifier.compare("format"))
            {
                if(line.substr(line.find_first_of(" ")+1).compare("ascii 1.0"))
                {
                    Logger::instance()->logX("sss","File format", 
                                             line.substr(line.find_first_of(" ")+1).c_str(), 
                                             "is not \"ascii 1.0\".");
                    file.close();
                    return;
                }
                continue;
            }
            
            // find vertex and face count
            if(!identifier.compare("element"))
            {
                std::string elem_type = 
                            line.substr(line.find_first_of(" ") + 1, 
                                             line.find_last_of(" ")
                                           - line.find_first_of(" ") - 1);
                if(!elem_type.compare("face"))
                {
                    m_facecount = atoi(line.substr(line.find_last_of(" ")+1).c_str());
                    Logger::instance()->logX("si","Face count is", m_facecount);
                }
                else if(!elem_type.compare("vertex"))
                {   
                    m_vertexcount = atoi(line.substr(line.find_last_of(" ")+1).c_str());
                    Logger::instance()->logX("si","Vertex count is", m_vertexcount);
                }
                continue;
            }
            
            // find property lists
            if(!identifier.compare("property"))
            {
                std::string elem_type = 
                            line.substr(line.find_first_of(" ") + 1, 4);
                // vertex property
                if(elem_type.compare("list"))
                {
                    std::string property_name = line.substr(line.find_last_of(" ")+1);
                    std::cout << "Property " << property_name << " added." << std::endl;
                    Vertex::add_property(property_name);                 
                }
                else // face property list
                {
                    // uninteresting for now
                }
                continue;
            }
        }
        
        // read vertices
        for(int i = 0, percent = 0, num_percent = m_vertexcount/100; 
            i < m_vertexcount; i++)
        {
            Vertex v = Vertex();
            getline(file, line);
            for(int p = 0; p < Vertex::property_count(); p++)
            {
                std::string value = line.substr(0, line.find_first_of(" "));
                line = line.substr(line.find_first_of(" ") + 1);
                v.set_value(Vertex::get_property(p), atof(value.c_str()));
            }
            m_vertices.push_back(v);
            
            if(i%num_percent == 0) 
            {
                std::cout << '\r' << "Adding vertices... " << ++percent << "%";
            }
        }
        std::cout << '\r' <<"                           " 
                  << '\r' << "Vertices done." << std::endl;
        
        // read faces
        for(int i = 0, percent = 0, num_percent = m_facecount/100; 
            i < m_facecount; i++)
        {
            Face f = Face();
            getline(file, line);
            int num_vertices = atoi(line.substr(0, line.find_first_of(" ")).c_str());
            line = line.substr(line.find_first_of(" ") + 1);
            for(int num = 0; num < num_vertices; num++)
            {
                int v = atoi(line.substr(0, line.find_first_of(" ")).c_str());
                line = line.substr(line.find_first_of(" ") + 1);
                f.add_vertex(&m_vertices.at(v));
            }
            m_faces.push_back(f);
            if(i%num_percent == 0) 
            {
                std::cout << '\r' << "Adding faces... " << ++percent << "%";
            }
        }
        std::cout << '\r' <<"                           " 
                  << '\r' << "Faces done." << std::endl;
        
    }
    else
    {
        Logger::instance()->log("Can not open file. Aborting.");
    }    
    file.close();
    Logger::instance()->logX("ss", "Done reading", m_filename.c_str());
}

std::string PolyMap::ltrim(std::string str)
{
    return str.erase(0, str.find_first_not_of(" "));
}

std::string PolyMap::rtrim(std::string str)
{
    return str.erase(str.find_last_not_of(" ") + 1);
}
std::string PolyMap::trim(std::string str)
{
    return ltrim(rtrim(str));
}
