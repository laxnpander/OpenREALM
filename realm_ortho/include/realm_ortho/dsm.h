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

#ifndef PROJECT_DSM_H
#define PROJECT_DSM_H

#include <memory>
#include <numeric>

#include <eigen3/Eigen/Eigen>
#include <opencv2/core.hpp>

#include <realm_types/enums.h>
#include <realm_types/cv_grid_map.h>
#include <realm_maths/plane_fitter.h>
#include <realm_ortho/nanoflann.h>
#include <realm_ortho/nearest_neighbor.h>

namespace realm
{

class DigitalSurfaceModel
{
  public:
    using Ptr = std::shared_ptr<DigitalSurfaceModel>;
    using ConstPtr = std::shared_ptr<const DigitalSurfaceModel>;

    // Wrapping typedefs in realm convention
    static constexpr size_t kMaxLeaf = 10u;
    static constexpr size_t kDimensionKdTree = 2u;
    using PointCloudAdaptor_t = PointCloudAdaptor<PointCloud<double>>;
    using KdTree_t = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Adaptor<double, PointCloudAdaptor_t>, PointCloudAdaptor_t, kDimensionKdTree>;
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
     */
    explicit DigitalSurfaceModel(const cv::Rect2d &roi);

    /*!
     * @brief Constructor for elevated surfaces, prior informations about the surface must have been computed.
     * Here: 3d point cloud
     * @param roi Region of interest of the observed surface, usually utm coordinates and width/height in [m]
     * @param points Cloud of observed surface points as Mat structured rowise: x, y, z
     * @param knn_radius_factor Factor for initial knn-search is GSD * knn_radius_factor
     */
    DigitalSurfaceModel(const cv::Rect2d &roi, const cv::Mat &points, SurfaceNormalMode mode, double knn_radius_factor);

    CvGridMap::Ptr getSurfaceGrid();

  private:

    //! Flag to set DSM as initialized
    bool _is_initialized;

    //! Flag to identify if prior normals should be used
    bool _use_prior_normals;

    //! Radius for nearest neighbour search
    double _knn_radius_factor;

    //! Assumption of the DSM. Either planar or elevation
    SurfaceAssumption _assumption;

    //! Mode of the surface normal computation
    SurfaceNormalMode _surface_normal_mode;

    //! Digital surface model of the input information
    CvGridMap::Ptr _surface;

    //! Input point cloud (only for elevation surface)
    PointCloud<double> _point_cloud;

    //! Adapter for point cloud k-d tree and NN search (only for elevation surface)
    std::unique_ptr<PointCloudAdaptor_t> _point_cloud_adaptor;

    //! Binary k-d tree for NN search. Only in xy-direction (only for elevation surface)
    std::unique_ptr<KdTree_t> _kd_tree;

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

} // namespace realm

#endif //PROJECT_DSM_H
