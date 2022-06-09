use crate::Point;
/// Abstract interface for map representation.
pub trait Map {
    fn get_occupancy(&self, position: Point) -> Option<bool>;
}
