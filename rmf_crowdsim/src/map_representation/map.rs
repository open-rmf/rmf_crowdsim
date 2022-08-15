use crate::Point;
/// Abstract interface for map representation.
pub trait Map {
    /// Returns true if occupied, false otherwise
    /// In the event the point is outside of the grid return None
    fn get_occupancy(&self, position: Point) -> Option<bool>;
}
