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
    m_filename = filename;
    // create hash map
    m_contents = std::map<std::string,std::string>();
    Logger::instance()->log("== Config File =====");
    Logger::instance()->logX("ss", "Config file:", m_filename.c_str());
    Logger::instance()->logX("sb", "Autoread is", autoread);
    // read file
    if(autoread)
    {
        if(this->read())
        {
            Logger::instance()->log("Autoread file.");
        }
    }
    Logger::instance()->log("");
}

ConfigFile::~ConfigFile()
{
}

bool ConfigFile::read()
{
    Logger::instance()->logX("ss", "Reading", m_filename.c_str());
    bool success = false;
    std::ifstream cfgfile(m_filename.c_str());
    if(cfgfile.good())
    {
        std::string line;
        while(getline(cfgfile, line))
        {
            line = trim(line);
            std::string key = line.substr(0, line.find(" "));
            std::string val = line.substr(line.find(" ") + 1);
            val = trim(val);
            m_contents.insert(std::pair<std::string, std::string>(key, val));
        }
    }
    else
    {
        Logger::instance()->log("Can not open file. Aborting.");
    }    
    cfgfile.close();
    Logger::instance()->logX("ss", "Done reading", m_filename.c_str());
    return success;
}

int ConfigFile::getAsInt(const std::string identifier, const int defaultValue)
{
    return m_contents.count(identifier)? atoi(m_contents[identifier].c_str()) : defaultValue;
}

float ConfigFile::getAsFloat(const std::string identifier, const float defaultValue)
{
    return getAsDouble(identifier, defaultValue);
}

double ConfigFile::getAsDouble(const std::string identifier, const double defaultValue)
{
    return m_contents.count(identifier)? atof(m_contents[identifier].c_str()) : defaultValue;
}

std::string ConfigFile::getAsString(const std::string identifier, const std::string defaultValue)
{
    return m_contents.count(identifier)? m_contents[identifier] : defaultValue;
}

float ConfigFile::getVecAsFloat(const std::string identifier, int index, const float defaultValue)
{
    if(!m_contents.count(identifier))
    {
        return defaultValue;
    }
    
    std::string val = m_contents[identifier];
    std::string nthval;
    for(int i = 0; i <= index; i++) {
        if(!val.compare("")) 
        {
            Logger::instance()->logX("sisss", 
                        "Requested index", index,
                        "does not exist in vector", identifier.c_str(),
                        ". Returning default.");
            return defaultValue;
        }
        nthval = val.substr(0, val.find_first_of(" "));
        val = ltrim(val.erase(0, val.find_first_of(" ")));
    }

    return atof(nthval.c_str());
}

std::string ConfigFile::ltrim(std::string str)
{
    return str.erase(0, str.find_first_not_of(" "));
}

std::string ConfigFile::rtrim(std::string str)
{
    return str.erase(str.find_last_not_of(" ") + 1);
}
std::string ConfigFile::trim(std::string str)
{
    return ltrim(rtrim(str));
}