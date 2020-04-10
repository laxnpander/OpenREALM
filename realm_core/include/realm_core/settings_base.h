/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PROJECT_SETTINGS_H
#define PROJECT_SETTINGS_H

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <unordered_map>

#include <realm_core/structs.h>

namespace realm
{

/*!
 * @brief Idea of this class is to provide a base for a wide range of settings files, e.g. for camera calibration,
 *        stage settings, implementations within the stage and everything else that might need settings somehow.
 *        Especially the ability to load a quickly defined settings class from a .yaml file is the main motivation
 *        for this implementation.
 *        The general usage is intended to work as follows:
 *        1) Derive a class from this one, e.g.
 *              class ExampleSettings : public SettingsBase
 *              {
 *                  ...
 *              };
 *        2) Add parameters to your special settings in the constructor. This is important because the ability to add
 *           Parameters is protected and only accessible for children of SettingsBase, e.g.
 *              ExampleSettings()
 *              {
 *                  add("a_param", Parameter_t<int>{"3", "This is a help description"});
 *              }
 *        3) Either create a settings file in code and set every parameter by hand (not recommended),
 *           or write a .yaml file containing all of the parameters of your ExampleSettings, give it valid values and
 *           load them in your code using "loadFromFile".
 *        4) Optional: Note that the base class is also able to sneak parameters from the file, if necessary. This can
 *           be quite useful, if you have several derivations from the SettingsBase. A good example is the camera settings.
 *           CameraSettings is derived from SettingsBase.
 *           PinholeSettings is derived from CameraSettings.
 *           You can now provide a factory class your general "CameraSettings" and dynamically check what parameter file
 *           was provided by the user to be loaded, e.g.
 *              std::string type = sneakParamFromFile<std::string>("type", "/path/to/yaml"))
 *              if (type == "pinhole")
 *                  do stuff;
 *              if (type == "fisheye")
 *                  do other stuff;
 *              ...
 */
class SettingsBase
{
  public:
    /*!
     * @brief Overloaded function to set a parameter of type int
     * @param key Parameter name to be set
     * @param val Value to be assigned to the parameter of name "param_name"
     */
    void set(const std::string &key, int val);

    /*!
     * @brief Overloaded function to set a parameter of type double
     * @param key Parameter name to be set
     * @param val Value to be assigned to the parameter of name "param_name"
     */
    void set(const std::string &key, double val);

    /*!
     * @brief Overloaded function to set a parameter of type string
     * @param key Parameter name to be set
     * @param val Value to be assigned to the parameter of name "param_name"
     */
    void set(const std::string &key, const std::string &val);

    /*!
     * @brief Basic functionality to load the settings from a .yaml file. For every parameter that was added in the
     *        derived class a parameter is searched for and set. If the parameter is not found, default values are kept.
     * @param filepath Absolute path to the .yaml file
     */
    void loadFromFile(const std::string &filepath);

    /*!
     * @brief Check if settings file has a certain parameter
     * @param param_name Name of the parameter to be checked
     * @return true if exists
     */
    bool has(const std::string &param_name);

    /*!
     * @brief Prints all parameters, their default values and the provided help description
     */
    void print();

    /*!
     * @brief Function to read parameters from a file. Can be used with the base class, so settings file can provide
     *        their own type as parameter (e.g. type: pinhole) and then be loaded dynamically.
     * @tparam T Type of the parameter to be sneaked
     * @param key Name of the parameter to be sneaked
     * @param filepath Absolute path to the settings file
     * @return Value of the parameter
     */
    template <typename T>
    T sneakParamFromFile(const std::string &key, const std::string &filepath)
    {
      cv::FileStorage fs(filepath, cv::FileStorage::READ);
      T val;
      if (fs.isOpened())
      {
        fs[key] >> val;
        set(key, val);
      }
      else
        throw std::invalid_argument("Error: Sneaking parameter from file " + filepath + " failed!");
      return val;
    }

    /*!
     * @brief Basic getter for value of key-value pair. Triggers overloaded functions
     * @tparam T Type of the parameter to be grabbed
     * @param key Name of the parameter value to be grabbed
     * @return value
     */
    template <typename T>
    T get(const std::string &key) const
    {
        Parameter_t<T> param{};
        get(key, &param);
        return param.value;
    }

  protected:
    /*!
     * @brief Add functionality for derived class to customize the settings. Can not be called from outside, to force
     *        explicit definition.
     * @param key Name of the parameter to be added
     * @param param Integer parameter containing value and description
     */
    void add(const std::string &key, const Parameter_t<int> &param);

    /*!
     * @brief Add functionality for derived class to customize the settings. Can not be called from outside, to force
     *        explicit definition.
     * @param key Name of the parameter to be added
     * @param param Double parameter containing value and description
     */
    void add(const std::string &key, const Parameter_t<double> &param);

    /*!
     * @brief Add functionality for derived class to customize the settings. Can not be called from outside, to force
     *        explicit definition.
     * @param key Name of the parameter to be added
     * @param param String parameter containing value and description
     */
    void add(const std::string &key, const Parameter_t<std::string> &param);

  private:

    //! Container for integer parameters
    std::unordered_map<std::string, Parameter_t<int> > _params_i;

    //! Container for double parameters
    std::unordered_map<std::string, Parameter_t<double> > _params_d;

    //! Container for string parameters
    std::unordered_map<std::string, Parameter_t<std::string> > _params_s;

    /*!
     * @brief Templated function to set a specific parameter. Called by overloaded public setter
     * @tparam T Type of the parameter to be set
     * @param map Container of the parameter type to be set
     * @param pair Parameter key-value pair
     * @throws out_of_range if parameter key is not found
     */
    template <typename T>
    void set(std::unordered_map<std::string, Parameter_t<T>> &map,
             const std::pair<std::string, T> &pair)
    {
      if (!map.empty())
      {
        auto it = map.find(pair.first);
        if (it == map.end())
          throw(std::out_of_range("Error: Setting parameter '" + pair.first + "' failed. Not found!"));
        else
          it->second.value = pair.second;
      }
      else
        throw(std::out_of_range("Error: Setting parameter failed. Container empty!"));
    }

    /*!
     * @brief Templated function to add a specific parameter. Called by overloaded, protected add functions. Adds
     *        parameter only if not already existing.
     * @tparam T Type of the parameter to be added
     * @param map Container to which parameter should be added (must match type)
     * @param pair Parameter key-value pair
     */
    template <typename T>
    void add(std::unordered_map<std::string, Parameter_t<T>> &map,
             const std::pair<std::string, Parameter_t<T>> &pair)
    {
        auto it = map.find(pair.first);
        if (it == map.end())
            map.insert({pair.first, pair.second});
        else
            map[pair.first] = pair.second;
    }

    /*!
     * @brief Private getter for parameter of key
     * @param key Name of the parameter
     * @param param Actual parameter as output containing value and help description
     * @throws out_of_range if parameter not found
     */
    void get(const std::string &key, Parameter_t<int> *param) const;

    /*!
     * @brief Private getter for parameter of key
     * @param key Name of the parameter
     * @param param Actual parameter as output containing value and help description
     * @throws out_of_range if parameter not found
     */
    void get(const std::string &key, Parameter_t<double> *param) const;

    /*!
     * @brief Private getter for parameter of key
     * @param key Name of the parameter
     * @param param Actual parameter as output containing value and help description
     * @throws out_of_range if parameter not found
     */
    void get(const std::string &key, Parameter_t<std::string> *param) const;
};

}

#endif //PROJECT_SETTINGS_H
