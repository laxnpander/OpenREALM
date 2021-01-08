

#ifndef PROJECT_DSM_H
#define PROJECT_DSM_H

#include <memory>
#include <numeric>

#include <eigen3/Eigen/Eigen>
#include <opencv2/core.hpp>

#include <realm_core/enums.h>
#include <realm_core/cv_grid_map.h>
#include <realm_core/plane_fitter.h>
#include <realm_ortho/nanoflann.h>
#include <realm_ortho/nearest_neighbor.h>

namespace realm
{
namespace ortho
{
class DigitalSurfaceModel
{
  public:
    using Ptr = std::shared_ptr<DigitalSurfaceModel>;
    using ConstPtr = std::shared_ptr<const DigitalSurfaceModel>;

    // Wrapping typedefs in realm convention
    static constexpr size_t kMaxLeaf = 10u;
    static constexpr size_t kDimensionKdTree = 2u;
    using PointCloudAdaptor_t = PointCloudAdaptor<ortho::PointCloud<double>>;
    using KdTree_t = nanoflann::KDTreeSingleIndexAdaptor<ortho::nanoflann::L2_Adaptor<double, PointCloudAdaptor_t>, PointCloudAdaptor_t, kDimensionKdTree>;
  public:
    enum class SurfaceNormalMode
    {
        NONE,
        RANDOM_NEIGHBOURS,
        FURTHEST_NEIGHBOURS,
        BEST_FIT
    };
  public:
    /*!
     * @brief Constructor for planar surfaces, no prior surface information exists
     * @param roi Region of interest of the observed surface, usually utm coordinates and width/height in [m]
     * @param elevation A fixed elevation can be set. This is especially useful, if the altitude of  the UAV is not measured
     * above ground.
     */
    explicit DigitalSurfaceModel(const cv::Rect2d &roi, double elevation = 0.0);

    /*!
     * @brief Constructor for elevated surfaces, prior informations about the surface must have been computed.
     * Here: 3d point cloud
     * @param roi Region of interest of the observed surface, usually utm coordinates and width/height in [m]
     * @param points Cloud of observed surface points as Mat structured rowise: x, y, z
     * @param knn_radius_factor Factor for initial knn-search is GSD * knn_radius_factor
     */
    DigitalSurfaceModel(const cv::Rect2d &roi, const cv::Mat &points, SurfaceNormalMode mode, int knn_max_iter);

    CvGridMap::Ptr getSurfaceGrid();

  private:

    //! Flag to identify if prior normals should be used
    bool m_use_prior_normals;

    //! Maximum iterations per grid cell to find the next nearest neigbour in the dense clouds.
    int m_knn_max_iter;

    //! Assumption of the DSM. Either planar or elevation
    SurfaceAssumption m_assumption;

    //! Mode of the surface normal computation
    SurfaceNormalMode m_surface_normal_mode;

    //! Digital surface model of the input information
    CvGridMap::Ptr m_surface;

    //! Input point cloud (only for elevation surface)
    ortho::PointCloud<double> m_point_cloud;

    //! Adapter for point cloud k-d tree and NN search (only for elevation surface)
    std::unique_ptr<PointCloudAdaptor_t> m_point_cloud_adaptor;

    //! Binary k-d tree for NN search. Only in xy-direction (only for elevation surface)
    std::unique_ptr<KdTree_t> m_kd_tree;

    /*!
     * @brief Initializes a k-d tree from the input point cloud. Important: k-d tree has 2 dimensions due to search in
     *        x- and y-direction only.
     * @param point_cloud Point cloud for which the k-d tree should be computed. Is structured as OpenCV mat type with
     *        row(i) = (x,y,z,r,g,b,nx,ny,nz)
     */
    void initKdTree(const PointCloud<double> &point_cloud);

    /*!
     * @brief Ground sampling distance for input point cloud is computed. Gives a guess on the resolution of the grid
     *        map. Only 100 of all points are used for computation to save time.
     * @param point_cloud Point cloud structured as OpenCV mat type with row(i) = (x,y,z,r,g,b,nx,ny,nz)
     * @return GSD that roughly describes the average minimum xy-distance for points of point cloud
     */
    double computePointCloudGSD(const PointCloud<double> &point_cloud);

    /*!
     * @brief Filter function for input point cloud. Currently not in use, but might be implemented later.
     * @param points Point cloud structured as OpenCV mat type with row(i) = (x,y,z,r,g,b,nx,ny,nz)
     * @return Point cloud structured as OpenCV mat type with row(i) = (x,y,z,r,g,b,nx,ny,nz)
     */
    cv::Mat filterPointCloud(const cv::Mat &points);

    /*!
     * @brief Main function to compute elevation grid map from an input point cloud. K-d tree must have been initialized
     *        beforhand.
     * @param point_cloud Point cloud structured as OpenCV mat type with row(i) = (x,y,z,r,g,b,nx,ny,nz)
     */
    void computeElevation(const cv::Mat &point_cloud);

    /*!
     * @brief Function to compute the surface normal based on the local neighbourhood. Several modi can be selected, like
     *        random neighbours, furthest neighbours or best-fit
     * @param points Vector of all neighboured points
     * @param dists Vector of distances to neighboured points
     * @return Surface normal for input points
     */
    cv::Vec3f computeSurfaceNormal(const std::vector<PlaneFitter::Point> &points, const std::vector<double> &dists);

    /*!
     * @brief Function to compute the indices of the k highest elements in the input vector
     * @param vec Vector with elements
     * @param k Number of k highest elements
     * @return Indices of k highest elements
     */
    std::vector<size_t> getKmaxElementsIndices(std::vector<double> vec, size_t k);

    /*!
     * @brief Function to interpolate height for a grid cell from a number of input points and their corresponding
     *        distances. Method is called "Inverse distance weighted" (IDW)
     * @param heights Height values of all neighboured points
     * @param dists Distance values of all neighboured points
     * @return Height interpolated
     */
    float interpolateHeight(const std::vector<double> &heights, const std::vector<double> &dists);

    /*!
     * @brief Function to interpolate surface normal for a grid cell from a number of input points and their corresponding
     *        normal. Method is called "Inverse distance weighted" (IDW)
     * @param normals All normals of the neighboured points
     * @param dists Distance values of all neighboured points
     * @return Interpolated surface normal
     */
    cv::Vec3f interpolateNormal(const std::vector<PlaneFitter::Normal> &normals, const std::vector<double> &dists);

    template <typename T>
    std::vector<size_t> sort_indices(const std::vector<T> &v)
    {
        // initialize original index locations
        std::vector<size_t> indices(v.size());
        std::iota(indices.begin(), indices.end(), 0);

        // sort indexes based on comparing values in v
        std::sort(indices.begin(), indices.end(),
             [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
        return indices;
    }
};
} // namespace ortho
} // namespace realm

#endif //PROJECT_DSM_H
