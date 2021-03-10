

#ifndef PROJECT_SETTINGS_H
#define PROJECT_SETTINGS_H

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <unordered_map>
#include <exception>

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
 *              std::string type = sneakParameterFromFile<std::string>("type", "/path/to/yaml"))
 *              if (type == "pinhole")
 *                  do stuff;
 *              if (type == "fisheye")
 *                  do other stuff;
 *              ...
 */
class SettingsBase
{
  public:
    class Variant
    {
      friend SettingsBase;
      enum class VariantType
      {
        INT,
        DOUBLE,
        STRING,
        MATRIX
      };
      using Ptr = std::unique_ptr<Variant>;
    public:
      explicit Variant(const Parameter_t<int> &p)         : m_type(VariantType::INT),    m_int_container(p)    {};
      explicit Variant(const Parameter_t<double> &p)      : m_type(VariantType::DOUBLE), m_double_container(p) {};
      explicit Variant(const Parameter_t<std::string> &p) : m_type(VariantType::STRING), m_string_container(p) {};
      explicit Variant(const Parameter_t<cv::Mat> &p)     : m_type(VariantType::MATRIX), m_mat_container(p)    {};

      int toInt() const
      {
        if (m_type == VariantType::INT) return m_int_container.value;
        if (m_type == VariantType::DOUBLE) return static_cast<int>(m_double_container.value);
        if (m_type == VariantType::STRING) throw(std::bad_cast());
        throw(std::bad_cast());
      }

      float toFloat() const
      {
        if (m_type == VariantType::INT) return static_cast<float>(m_int_container.value);
        if (m_type == VariantType::DOUBLE) return static_cast<float>(m_double_container.value);
        if (m_type == VariantType::STRING) throw(std::bad_cast());
        throw(std::bad_cast());
      }

      double toDouble() const
      {
        if (m_type == VariantType::INT) return static_cast<double>(m_int_container.value);
        if (m_type == VariantType::DOUBLE) return m_double_container.value;
        if (m_type == VariantType::STRING) throw(std::bad_cast());
        throw(std::bad_cast());
      }

      std::string toString() const
      {
        if (m_type == VariantType::INT) return std::to_string(m_int_container.value);
        if (m_type == VariantType::DOUBLE) return std::to_string(m_double_container.value);
        if (m_type == VariantType::STRING) return m_string_container.value;
        throw(std::bad_cast());
      }

      cv::Mat toMat() const
      {
        if (m_type != VariantType::MATRIX) throw(std::bad_cast());
        return m_mat_container.value;
      }

      std::string help() const
      {
        if (m_type == VariantType::INT)    return m_int_container.help;
        if (m_type == VariantType::DOUBLE) return m_double_container.help;
        if (m_type == VariantType::STRING) return m_string_container.help;
        if (m_type == VariantType::MATRIX) return m_mat_container.help;
        return m_string_container.help;
      }

    private:
      VariantType m_type;
      Parameter_t<std::string> m_string_container;
      Parameter_t<double> m_double_container;
      Parameter_t<int> m_int_container;
      Parameter_t<cv::Mat> m_mat_container;
    };

  public:

    Variant operator[](const std::string &key) const;

    Variant get(const std::string &key) const;

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
   * @brief Overloaded function to set a parameter of type cv::Mat
   * @param key Parameter name to be set
   * @param val Value to be assigned to the parameter of name "param_name"
   */
  void set(const std::string &key, const cv::Mat &val);

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
    bool has(const std::string &param_name) const;

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
    static T sneakParameterFromFile(const std::string &key, const std::string &filepath)
    {
      cv::FileStorage fs(filepath, cv::FileStorage::READ);
      T val;
      if (fs.isOpened())
      {
        fs[key] >> val;
      }
      else
        throw std::invalid_argument("Error: Sneaking parameter from file " + filepath + " failed!");
      return val;
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

  /*!
   * @brief Add functionality for derived class to customize the settings. Can not be called from outside, to force
   *        explicit definition.
   * @param key Name of the parameter to be added
   * @param param String parameter containing value and description
   */
  void add(const std::string &key, const Parameter_t<cv::Mat> &param);

  private:

    //! Container for parameters
    std::unordered_map<std::string, Variant::Ptr> m_parameters;
};

}

#endif //PROJECT_SETTINGS_H
