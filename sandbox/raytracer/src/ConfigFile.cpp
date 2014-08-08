/**
 * ConfigFile
 * @description a config file for the camera parameters
 * @author Sebastian Höffner
 * @version 0.1
 * @created 2014-08-08
 * @lastModification 2014-08-08
 * @file ConfigFile.cpp
 */
 
#include "ConfigFile.h"

ConfigFile::ConfigFile(std::string filename, bool autoread)
{
    // set file name
    this->m_filename = filename;
    // create hash map
    //this->m_contents = new map<std::string,std::string>();
    Logger::instance()->log("== Config File =====");
    Logger::instance()->logX("ss", "Config file:", filename);
    Logger::instance()->logX("sb", "Autoread is", autoread);
    // read file
    if(autoread) 
    {
        this->read();
    }
}

ConfigFile::~ConfigFile()
{
    // if(m_contents != NULL)
    // {
        // delete m_contents;
        // m_contents = NULL;
    // }
}

bool ConfigFile::read() const
{
    Logger::instance()->logX("ss", "Reading", filename);
    bool success = false;
    
    
    
    Logger::instance()->logX("ss", "Done reading", filename);
    return success;
}

int ConfigFile::getAsInt(const std::string identifier, const int defaultValue) const
{
    return 0;
}

float ConfigFile::getAsFloat(const std::string identifier, const float defaultValue) const
{
    return 0;
}

double ConfigFile::getAsDouble(const std::string identifier, const float defaultValue) const
{
    return 0;
}

std::string ConfigFile::getAsString(const std::string identifier, const std::string defaultValue) const
{
    return "";
}