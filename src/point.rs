use std::ops::{Sub, Div};

#[cfg(use_int32)]
pub type CInt = i32;
#[cfg(not(use_int32))]
pub type CInt = i64;

pub trait Coords<T>: PartialEq + Copy + Clone
{
    fn new(x: T, y: T) -> Self;
    fn get_x(&self) -> T;
    fn get_y(&self) -> T;
    fn set_x(&mut self, x: T);
    fn set_y(&mut self, y: T);
}

pub trait DoubleCoords: Coords<f64> {
}

#[derive(Copy, Clone, PartialEq)]
#[repr(packed)]
pub struct Point<T> {
  pub x: T,
  pub y: T,
}
/*
impl<T> Point<T> {
    fn new(x: T, y: T) -> Self { Self { x: x, y: y } }
    fn get_x(&self) -> T { self.x }
    fn get_y(&self) -> T { self.y }
    fn set_x(&mut self, x: T) { self.x = x; }
    fn set_y(&mut self, y: T) { self.y = y; }
}
*/


#[derive(Copy, Clone, PartialEq)]
#[repr(packed)]
pub struct DoublePoint {
  pub x: f64,
  pub y: f64,
}

impl Coords<f64> for DoublePoint {
    fn new(x: f64, y: f64) -> Self { Self { x: x, y: y } }
    fn get_x(&self) -> f64 { self.x }
    fn get_y(&self) -> f64 { self.y }
    fn set_x(&mut self, x: f64) { self.x = x; }
    fn set_y(&mut self, y: f64) { self.y = y; }
}

impl DoublePoint {
    fn get_dx(&self, other: &Self) -> f64 {
        if self.get_y() == other.get_y() {
            ::consts::HORIZONTAL
        } else {
            (other.get_x() - self.get_x()) as f64 /
                (other.get_y() - self.get_y()) as f64
        }
    }
}
