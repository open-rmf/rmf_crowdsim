use crate::spatial_index::spatial_index::SpatialIndex;

struct LocationHash2D{
    data: std::Vec<std::HashSet>,
    width: f64,
    height: f64,
    /// Resolution of width of each cell
    x_resolution: f64,
    /// Resulution of height of each cell
    y_resolution: f64
};

