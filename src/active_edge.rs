use point::{Point, Coords, DoublePoint};
use {EdgeSide, PolyType};
use OutRec;

pub struct ActiveEdge {
    pub bot: DoublePoint,
    /// current (updated for every new scanbeam)
    pub cur: DoublePoint,
    pub top: DoublePoint,
    pub delx: f64,
    pub poly_typ: PolyType,
    /// side only refers to current side of solution poly
    pub side: EdgeSide,
    /// 1 or -1 depending on winding direction
    pub wind_delta: u8,
    pub wind_cnt: isize,
    //winding count of the opposite polytype
    pub winding_count_2: isize,
    pub out_rec: OutRec,
    pub next: Box<ActiveEdge>,
    pub prev: Box<ActiveEdge>,
    pub next_in_lml: Box<ActiveEdge>,
    pub next_in_ael: Box<ActiveEdge>,
    pub prev_in_ael: Box<ActiveEdge>,
    pub next_in_sel: Box<ActiveEdge>,
    pub prev_in_sel: Box<ActiveEdge>,
}

impl ActiveEdge {
    #[inline]
    pub fn is_horizontal(&self) -> bool {
        self.delx == ::consts::HORIZONTAL
    }

    #[inline]
    pub fn set_delx(&mut self) {
        let dy  = self.top.get_y() - self.bot.get_y();
        self.delx = if dy == 0f64 {
            ::consts::HORIZONTAL
        } else {
            (self.top.get_x() - self.bot.get_x()) as f64 / dy as f64
        };
    }

}

#[inline]
pub fn swap_sides(ae1: &mut ActiveEdge, ae2: &mut ActiveEdge) {
    ::std::mem::swap(&mut ae1.side, &mut ae2.side);
}

#[inline]
pub fn top_x(ae: &ActiveEdge, y: f64) -> f64 {
    if y == ae.top.get_y() {
        ae.top.get_x()
    } else if y == ae.bot.get_y(){
        ae.bot.get_x()
    }
    else {
        ae.bot.get_x() + ae.delx * (y - ae.bot.get_y())
    }
}

/// Calculates the intersection point between two edges
pub fn intersect_edges(ae1: &ActiveEdge, ae2: &ActiveEdge) -> DoublePoint {

    let mut ip: DoublePoint;

    // warn: matching floating point value
    if ae1.delx == ae2.delx {
        let cy = ae1.cur.get_y();
        return DoublePoint::new(top_x(ae1, cy), cy);
    }

    // warn: matching floating point value
    else if ae1.delx == 0.0 {
        let cur_y = if ae2.is_horizontal() {
            ae2.bot.get_y()
        } else {
            let b2 = ae2.bot.get_y() as f64 - (ae2.bot.get_x() as f64 / ae2.delx);
            ae1.bot.get_x() as f64 / ae2.delx + b2
        };

        ip = DoublePoint::new(ae1.bot.get_x(), cur_y);
    }

    // warn: matching floating point value
    // reverse of previous block
    else if ae2.delx == 0.0 {
        let cur_y = if ae1.is_horizontal() {
            ae1.bot.get_y()
        } else {
            let b2 = ae1.bot.get_y() as f64 - (ae1.bot.get_x() as f64 / ae1.delx);
            ae2.bot.get_x() as f64 / ae1.delx + b2
        };

        ip = DoublePoint::new(ae2.bot.get_x(), cur_y);
    }

    else {
        let b1 = (ae1.bot.get_x() - ae1.bot.get_y()) as f64 * ae1.delx;
        let b2 = (ae2.bot.get_x() - ae2.bot.get_y()) as f64 * ae2.delx;
        let q = (b2 - b1) as f64 / (ae1.delx - ae2.delx);
        let cur_y = q;
        let cur_x = if ae1.delx.abs() < ae2.delx.abs() {
            ae1.delx * q + b1
        } else {
            ae2.delx * q + b2
        };

        ip = DoublePoint::new(cur_x, cur_y);
    }

    if ip.get_y() > ae1.cur.get_y() {
        let prev_y = ae1.cur.get_y();
        ip.set_y(prev_y);
        if ae1.delx.abs() > ae2.delx.abs() {
            ip.set_x(top_x(ae2, prev_y));
        } else {
            ip.set_x(top_x(ae1, prev_y));
        }
    }

    ip
}
#[inline]
#[cfg(all(use_int32, use_int128))]
pub fn slopes_equal_edge2<T>(e1: &ActiveEdge<Point<T>>, e2: &ActiveEdge<Point<T>>) -> bool {
    let sdy = e1.top.get_y() - e1.bot.get_y();
    let sdx = e1.top.get_x() - e1.bot.get_x();
    let edy = e2.top.get_y() - e2.bot.get_y();
    let edx = e2.top.get_x() - e2.bot.get_x();

    let a: f64 = sdy * edx;
    let b: f64 = sdx * edy;
    a == b
}

#[inline]
#[cfg(not(all(use_int32, use_int128)))]
pub fn slopes_equal_edge2(e1: &ActiveEdge, e2: &ActiveEdge) -> bool {
    let sdy = e1.top.get_y() - e1.bot.get_y();
    let sdx = e1.top.get_x() - e1.bot.get_x();
    let edy = e2.top.get_y() - e2.bot.get_y();
    let edx = e2.top.get_x() - e2.bot.get_x();
    sdy * edx == sdx * edy
}

#[inline]
#[cfg(all(use_int32, use_int128))]
pub fn slopes_equal_point3(p1: &DoublePoint, p2: &DoublePoint, p3: &DoublePoint) -> bool {
    let p12y = p1.get_y() - p2.get_y();
    let p12x = p1.get_x() - p2.get_x();
    let p23y = p2.get_y() - p3.get_y();
    let p23x = p2.get_x() - p3.get_x();

    let a: f64 = p12y * p23x;
    let b: f64 = p12x * p23y;
    a == b
}

#[inline]
#[cfg(not(all(use_int32, use_int128)))]
pub fn slopes_equal_point3(p1: &DoublePoint, p2: &DoublePoint, p3: &DoublePoint) -> bool {
    let p12y = p1.get_y() - p2.get_y();
    let p12x = p1.get_x() - p2.get_x();
    let p23y = p2.get_y() - p3.get_y();
    let p23x = p2.get_x() - p3.get_x();
    p12y * p23x == p12x * p23y
}

#[inline]
#[cfg(all(use_int32, use_int128))]
pub fn slopes_equal_point4(p1: &DoublePoint, p2: &DoublePoint, p3: &DoublePoint, p4: &DoublePoint) -> bool {
    let p12y = p1.get_y() - p2.get_y();
    let p12x = p1.get_x() - p2.get_x();
    let p34y = p3.get_y() - p4.get_y();
    let p34x = p3.get_x() - p4.get_x();
    let a: f64 = p12y * p34x;
    let b: f64 = p12x * p34y;
    a == b
}

#[inline]
#[cfg(not(all(use_int32, use_int128)))]
pub fn slopes_equal_point4(p1: &DoublePoint, p2: &DoublePoint, p3: &DoublePoint, p4: &DoublePoint) -> bool {
    let p12y = p1.get_y() - p2.get_y();
    let p12x = p1.get_x() - p2.get_x();
    let p34y = p3.get_y() - p4.get_y();
    let p34x = p3.get_x() - p4.get_x();
    p12y * p34x == p12x * p34y
}
