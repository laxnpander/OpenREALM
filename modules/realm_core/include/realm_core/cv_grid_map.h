

#ifndef PROJECT_CV_GRID_MAP_H
#define PROJECT_CV_GRID_MAP_H

#include <vector>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace realm
{

/*!
* @brief Enumerations for matrix merge function.
* @var REALM_OVERWRITE_ALL Input matrix overwrites all elements of the destination matrix
* @var REALM_OVERWRITE_ZERO Input matrix overwrites all zero elements or NAN of the destination matrix
*/
enum
{
  REALM_OVERWRITE_ALL,
  REALM_OVERWRITE_ZERO,
};

/*!
 * @brief Idea adapted from Péter Frankhauser's GridMap library. However due to overhead and performance issues this
 * slim intermediate class was designed to provide an easy and efficient interface for the basic realm package
 * functionalities. It is mainly used for storage and conversion.
 */

class CvGridMap
{
  public:
    using Ptr = std::shared_ptr<CvGridMap>;
    using ConstPtr = std::shared_ptr<const CvGridMap>;

    using Overlap = std::pair<CvGridMap::Ptr, CvGridMap::Ptr>;
    struct Layer
    {
        std::string name;
        cv::Mat data;
        int interpolation;
    };

  public:
    /*!
     * @brief Default constructor
     */
    CvGridMap();

    /*!
     * @brief Constructor to create a CvGridMap and setGeometry in the same step.
     */
    CvGridMap(const cv::Rect2d &roi, double resolution);

    /*!
     * @brief Function to create deep copy of CvGridMap
     * @return Deep copy of this object
     */
    CvGridMap clone() const;

    /*!
     * @brief Function to create deep copy of specific layers of this object
     * @param layer_names Vector of layer names
     * @return Deep copy of this object with specified layers only
     */
    CvGridMap cloneSubmap(const std::vector<std::string> &layer_names);

    /*!
     * @brief Adds a layer with name and data to the appropriate container
     * @param layer layer consisting of a name and data
     * @param is_data_empty Flag to allow adding empty matrices and later set the content
     * Currently float, double, CV8UC as data mat is supported
     */
    void add(const Layer &layer, bool is_data_empty = false);

    /*!
     * @brief Adds a layer with name and data to the appropriate container with specified interpolation for that layer
     * @param layer_name name of the layer, e.g. "elevation"
     * @param layer_data data of the layer: float, double, CV8UC as data mat is supported
     * @param interpolation interpolation flag for opencv, e.g. CV_INTER_LINEAR
     * @return
     */
    void add(const std::string &layer_name, const cv::Mat &layer_data, int interpolation = cv::InterpolationFlags::INTER_LINEAR);

    /*!
     * @brief Adds a submap to the existing grid map
     * Note: size do not have to match up
     * @param submap
     */
    void add(const CvGridMap &submap, int flag_overlap_handle, bool do_extend = true);

    /*!
     * @brief Removes a layer from the grid
     * @param layer_name Name of the layer to be removed
     */
    void remove(const std::string &layer_name);

    /*!
    * @brief Removes layers from the grid
    * @param layer_name Vector of names of layers to be removed
    */
    void remove(const std::vector<std::string> &layer_names);

    /*!
     * @brief Sets the geometry of the grid map, therefore size of the grid (nrof rows, nrof cols),
     * resolution and the world ROI with (x,y,width,height) in [meters]
     * @param roi Region of interest in the world frame, in aerial mapping typically utm position and dimension
     * @param resolution Resolution as [m/cell], typically also referred to as ground sampling distance (GSD) in aerial
     * mapping
     */
    void setGeometry(const cv::Rect2d &roi, double resolution);

    /*!
     * @brief Setter for interpolation flag of a specific layer
     * @param layer_name Name of the layer for which the interpolation should be set
     * @param interpolation OpenCV interpolation flag, e.g. CV_INTER_LINEAR
     */
    void setLayerInterpolation(const std::string& layer_name, int interpolation);

    /*!
     * @brief Extends the existing region of interest and therefore grid size by a desired other roi
     * @param roi region of interest that should add up to the existing one. New region of interest is then the
     * enclosing rectangle of both rois
     */
    void extendToInclude(const cv::Rect2d &roi);

    /*!
     * @brief Function to change resolution of the grid map. Every data layer gets resized with opencv resize function
     * according to the new resolution. Beware: roi might also change to fit the new dimensions adequatly.
     * @param resolution New resolution to be achieved
     */
    void changeResolution(double resolution);

    /*!
     * @brief Function to change resolution of the grid map. It enforces a desired matrix size rather than changing
     * the resolution to fit some desired size.
     * @param size Size, that the data matrices should have after resizing. Resulting resize operation must be uniformly in x and y.
     */
    void changeResolution(const cv::Size2i &size);

    /*!
     * @brief Iterates through container to find layer by name and returns true if found
     * @param layer_name name of the desired layer
     * @return true if found, false if not existend
     */
    bool exists(const std::string &layer_name) const;

    /*!
     * @brief opencv style function to check wether the object has already been "used" (got data) before or not
     * @return true if setGeometry was called at least once
     */
    bool empty() const;

    /*!
     * @brief Calculates, if a defined roi in the world frame is completly contained in the current map roi
     * @param roi defined roi in the world frame
     * @return true if it is contained, false if not
     */
    bool containsRoi(const cv::Rect2d &roi) const;

    /*!
     * @brief Iterates through container to find layer and returns the data when found, exception when non existend
     * @param layer_name name of the desired layer
     * @return data matrix of the desired layer: float, double, CV8UC as data mat is supported
     * @throws out_of_range if layer not existend, data can not be set. Exception out_of_range is thrown
     */
    cv::Mat& get(const std::string &layer_name);

    /*!
     * @brief Iterates through container to find layer and returns the data when found, exception when non existend
     * @param layer_name name of the desired layer
     * @return data matrix of the desired layer: float, double, CV8UC as data mat is supported
     * @throws out_of_range if layer does not exist, data can not be set. Exception out_of_range is thrown
     */
    const cv::Mat& get(const std::string &layer_name) const;

    /*!
     * @brief Overloaded [] operator for fast accessing layer data elements, e.g. map["color"].at<cv::Vec3b>(r, c).
     * Be careful: if you use as assignment like map["color"] = cv::Mat(...) the size of the mat can not be cross checked
     * for size. If the size does not match the CvGridMap size, you might break it.
     * @param layer_name name of the desired layer
     * @return data matrix of the desired layer: float, double, CV8UC as data mat is supported
     * @throws out_of_range if layer does not exist, data can not be set. Exception out_of_range is thrown
     */
    cv::Mat& operator[](const std::string& layer_name);

    /*!
     * @brief Overloaded [] operator for fast accessing layer data elements, e.g. map["color"].at<cv::Vec3b>(r, c)
     * @param layer_name name of the desired layer
     * @return data matrix of the desired layer: float, double, CV8UC as data mat is supported
     * @throws out_of_range if layer not existend, data can not be set. Exception out_of_range is thrown
     */
    const cv::Mat& operator[](const std::string& layer_name) const;

    /*!
     * @brief Function to grab a whole layer including name and interpolation flag
     * @param layer_name Name of the layer to be grabbed
     * @return Layer
     */
    Layer getLayer(const std::string& layer_name) const;

    /*!
     * @brief Iterates through all existing layers and extracts their names
     * @return vector of all layer names
     */
    std::vector<std::string> getAllLayerNames() const;

    /*!
     * @brief Extracts layers from the grid map as a new grid map
     * @param layer_names names of the desired layers within the grid map
     * @return submap of desired layers
     */
    CvGridMap getSubmap(const std::vector<std::string> &layer_names) const;

    /*!
     * @brief Extracts a floating point region of interest from specified layers
     * @param layer_names names of the desired layers of the submap
     * @param roi floating point region of interest to be extracted
     * @return submap of roi with desired layers
     */
    CvGridMap getSubmap(const std::vector<std::string> &layer_names, const cv::Rect2d &roi) const;

    /*!
     * @brief Extracts a index-based region of interest from specified layers
     * @param layer_names names of the desired layers of the submap
     * @param roi index-based region of interest to be extracted
     * @return submap of roi with desired layers
     */
    CvGridMap getSubmap(const std::vector<std::string> &layer_names, const cv::Rect2i &roi) const;

    /*!
     * @brief Extracts the overlapping region of two CvGridMaps and returns it as an isolated CvGridMap
     * @param other_map Other grid map to be compared with
     * @return Overlapping region as submap
     */
    Overlap getOverlap(const CvGridMap &other_map) const;

    /*!
     * @brief Accessing 2d index in the grid at a position in the world frame
     * @param pos 2d world position, typically in utm32-coordinates for aerial mapping
     * @return 2d index of world position in the grid map
     * @throws out_of_range if index is outside the grid an exception out_of_range is thrown
     */
    cv::Point2i atIndex(const cv::Point2d &pos) const;

    /*!
     * @brief Accessing a region of interest in the world frame within the grid map matrix
     * @param roi Region of interest in the world frame
     * @return Region of interest in the matrix
     */
    cv::Rect2i atIndexROI(const cv::Rect2d &roi) const;

    /*!
     * @brief Accessing the 2d world position of a grid element. Typically these are utm32-coordinates in aerial mapping
     * @param r row in the data matrix
     * @param c col in the data matrix
     * @return x,y-position in the world frame
     */

    cv::Point2d atPosition2d(uint32_t r, uint32_t c) const;

    /*!
     * @brief Accessing the 3d world position of a grid element. Typically these are utm32-coordinates in aerial mapping
     * for x,y and different types of data for z (elevation, observation angle, ...)
     * @param layer_name name of the desired layer for the z-data
     * @param r row in the data matrix
     * @param c col in the data matrix
     * @return x,y,z-position in the world frame
     */
    cv::Point3d atPosition3d(const int &r, const int &c, const std::string &layer_name) const;

    /*!
     * @brief Getter for the current resolution of the grid
     * @return resolution of the grid
     */
    double resolution() const;

    /*!
     * @brief Getter for the current size of the grid
     * @return size of the grid
     */
    cv::Size2i size() const;

    /*!
     * @brief Getter for the current roi of the grid
     * @return roi of the grid
     */
    cv::Rect2d roi() const;

private:
    // resolution therefor [m] / cell
    double m_resolution;
    // Global region of interest the map is played in
    cv::Rect2d m_roi;
    // Dimensions of the grid in [cols, rows]
    cv::Size2i m_size;
    // vector of all layers added, currently very dynamic operations
    // like adding and removing layers frequently is not expected
    std::vector<Layer> m_layers;

    void mergeMatrices(const cv::Mat &mat1, cv::Mat &mat2, int flag_merge_handling);

  /*!
     * @brief Checks the input matrix type and return true if CvGridMap currently supports it.
     * @param type OpenCV matrix type, e.g. CV_32F, ...
     * @return True if matrix type is supported
     */
    bool isMatrixTypeValid(int type);

    /*!
     * @brief Function to find the idx of a layer inside the layer container
     * @param layer_name Name of the layer to be found
     * @return idx of the layer inside vector "layers"
     */
    uint32_t findContainerIdx(const std::string &layer_name);

    /*!
     * @brief Function to fit the desired roi to a valid number.
     * @param roi_desired Input; Desired region of interest called by "setGeometry(...)"
     * @param roi_set Output; Region of interest that was actually set. In most cases it is not equal to the desired
     *        region of interest, but fitted to the closest resolution sample
     * @param size_set Output; Depends on the roi set, is the actual matrix size / grid size depending on the resolution
     */
    void fitGeometryToResolution(const cv::Rect2d &roi_desired, cv::Rect2d &roi_set, cv::Size2i &size_set);

    /*!
     * @brief This function allows to round up a floating point value to the nearest value, which is dividable through the
     * resolution. Example: roundToResolution(6.4, 2.0) = 8.0
     * @param value value to be rounded
     * @param resolution resolution of the operation
     * @return up rounded value
     */
    static double roundToResolution(double value, double resolution);
};

}

#endif //PROJECT_CV_GRID_MAP_H
