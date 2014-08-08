/**
 * ConfigFile Header
 * @description a config file for the camera parameters
 * @author Sebastian Höffner
 * @created 2014-08-08
 * @lastModification 2014-08-08
 * @file ConfigFile.h
 */
#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include "Logger.h"

/**
 * ConfigFile for CameraParameters
 */
class ConfigFile {
public:
	/**
	 * default constructor
     * @param filename file name to read
     * @param autoread set true to call ConfigFile::read() automatically
	 */
	ConfigFile(std::string filename, bool autoread  = false);

    /**
     * destructor
     */ 
	~ConfigFile();

    /**
     * Reads the file and stores its lines in a hash map to be retrieved later.
     * @return true on success, false on errors
     */
	bool read();

	/**
     * Looks up the identifier in the hash map and tries to parse it as int.
     * Returns the parsed value on success and the default value otherwise.
	 * @param identifier value identifier in config file, i.e. in hash map
     * @param defaultValue value to return in case there is an error.
	 * @return the value from the hash map or the default value
	 */
    int getAsInt(const std::string identifier, const int defaultValue);
    
	/**
     * Looks up the identifier in the hash map and tries to parse it as float.
     * Returns the parsed value on success and the default value otherwise.
	 * @param identifier value identifier in config file, i.e. in hash map
     * @param defaultValue value to return in case there is an error.
	 * @return the value from the hash map or the default value
	 */
    float getAsFloat(const std::string identifier, const float defaultValue);
    
	/**
     * Looks up the identifier in the hash map and tries to parse it as double. 
     * Returns the parsed value on success and the default value otherwise.
	 * @param identifier value identifier in config file, i.e. in hash map
     * @param defaultValue value to return in case there is an error.
	 * @return the value from the hash map or the default value
	 */
    double getAsDouble(const std::string identifier, const double defaultValue);
    
	/**
     * Looks up the identifier in the hash map and tries to parse it as std::string. 
     * Returns the parsed value on success and the default value otherwise.
	 * @param identifier value identifier in config file, i.e. in hash map
     * @param defaultValue value to return in case there is an error.
	 * @return the value from the hash map or the default value
	 */
    std::string getAsString(const std::string identifier, const std::string defaultValue);
    
    /**
     * Looks up the identifier in the hash map and tries to parse it as floats. 
     * Returns the parsed value on success and the default value otherwise.
	 * @param identifier value identifier in config file, i.e. in hash map
     * @param index access this value's index, 0-based
     * @param defaultValue value to return in case there is an error.
	 * @return the value from the hash map or the default value
	 */
    float getVecAsFloat(const std::string identifier, int index, const float defaultValue);

private:
    std::map<std::string, std::string> m_contents;
    std::string m_filename;
    
    /**
     * Trims whitespace off the beginning of a string.
     */
    std::string ltrim(std::string str);
    
    /**
     * Trims whitespace off the end of a string.
     */
    std::string rtrim(std::string str);
    
    /**
     * Trims whitespace off a string.
     */
    std::string trim(std::string str);
};

#endif // CONFIGFILE_H
