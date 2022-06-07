use rmf_crowdsim::{Map, HighLevelPlanner, Point};
use nannou::prelude::*;

struct NaiveMap {}

impl Map for NaiveMap
{
    fn get_occupancy(&self, _pt :Point) -> Option<bool>
    {
        return Some(true);
    }
}

fn main() {
    nannou::sketch(view).run()
}

fn view(app: &App, frame: Frame) {
    // Begin drawing
    let draw = app.draw();

    // Clear the background to blue.
    draw.background().color(CORNFLOWERBLUE);
}
