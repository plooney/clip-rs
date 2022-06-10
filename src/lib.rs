//! # clip-rs
//!
//! Rust library for polygon clipping by Padraig Looney
//!
//! With thanks to work by Angus Johnson's Clipper and Clipper 2 and Felix Schutt's 
//! port of Clipper to Rust
//!
//! # About this library
//!
//! The code in this library is an extension of Bala Vatti's clipping algorithm:
//! "A generic solution to polygon clipping"
//! Communications of the ACM, Vol 35, Issue 7 (July 1992) pp 56-63.
//! http://portal.acm.org/citation.cfm?id=129906
//!
//! Computer graphics and geometric modeling: implementation and algorithms
//! By Max K. Agoston
//! Springer; 1 edition (January 4, 2005)
//! http://books.google.com/books?q=vatti+clipping+agoston
//!
//! See also:
//! "Polygon Offsetting by Computing Winding Numbers"
//! Paper no. DETC2005-85513 pp. 565-575
//! ASME 2005 International Design Engineering Technical Conferences
//! and Computers and Information in Engineering Conference (IDETC/CIE2005)
//! September 24-28, 2005 , Long Beach, California, USA
//!
//! http://www.me.berkeley.edu/~mcmains/pubs/DAC05OffsetPolygon.pdf

// stdsimd needs this for detecting CPU features at runtime
//#![feature(cfg_target_feature)]

#![allow(dead_code)]
#![allow(unused_macros)]

#[macro_use]
extern crate ordered_float;
use ordered_float::OrderedFloat;
pub mod macros;
pub mod consts;
pub mod active_edge;
pub mod node;
pub mod point;

use std::{error::Error, fmt};
use std::collections::{BTreeMap, HashMap, LinkedList};
use std::fmt::Display;
use std::marker::PhantomData;
use std::sync::Arc;

use point::{Coords, DoublePoint};
use node::PolyNode;
use consts::*;
use active_edge::ActiveEdge;

/// By far the most widely used winding rules for polygon filling are
/// EvenOdd & NonZero (GDI, GDI+, XLib, OpenGL, Cairo, AGG, Quartz, SVG, Gr32)
/// Others rules include Positive, Negative and ABS_GTR_EQ_TWO (only in OpenGL)
/// see http://glprogramming.com/red/chapter11.html
#[derive(PartialEq, Eq)]
pub enum PolyFillType {
    EvenOdd,
    NonZero,
    Positive,
    Negative,
}

#[derive(PartialEq, Eq)]
pub enum ClipType {
    Intersection,
    Union,
    Difference,
    Xor,
}

#[derive(PartialEq, Eq)]
pub enum PolyType {
    Subject,
    Clip,
}

#[derive(PartialEq, Eq, Copy, Clone)]
pub enum EndType {
    ClosedPolygon,
    ClosedLine,
    OpenButt,
    OpenSquare,
    OpenRound
}

#[derive(PartialEq, Eq)]
pub enum EdgeSide {
    Left,
    Right,
}

#[derive(PartialEq, Eq)]
pub enum Direction {
    RightToLeft,
    LeftToRight,
}

#[derive(PartialEq, Eq, Copy, Clone)]
pub enum InitOptions {
    ReverseSolution,
    StrictlySimple,
    PreserveCollinear,
}

#[derive(PartialEq, Eq, Copy, Clone)]
pub enum JoinType {
    Square,
    Round,
    Miter,
}

#[derive(Debug)]
pub struct ClipErr;
impl Error for ClipErr {}
impl Display for ClipErr {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Error in ClipRs")
    }
}

pub type Path = Vec<DoublePoint>;
pub type Ind = i32;
pub type Paths = Vec<Path>;
pub type LocalMinima = HashMap<Ind,Path>;
pub type Edges = Vec<ActiveEdge>;
pub type Scanline = f64;
pub type VertexFlags = Vec<VertexFlag>;
pub type Edge = (DoublePoint, DoublePoint);

// In Rust you can't have pointers like in the C++ version
// So we use indices instead

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct PolyNodeIndex {
    pub(crate) node_idx: usize
}

/*
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct EdgeIndex {
    pub(crate) edge_idx: usize
}
*/

pub struct PolyTree {
    /// Pool of nodes
    pub nodes: Vec<PolyNode>,
    pub edges: Vec<ActiveEdge>,
}

pub struct PolyPath {
    /// Pool of nodes
    pub nodes: Vec<PolyNode>,
    pub edges: Vec<ActiveEdge>,
}

impl PolyTree {

    /// Creates a new, empty PolyTree
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
        }
    }

    pub fn get_first(&self) -> Option<&PolyNode> {
        self.nodes.get(0)
    }

    pub fn total(&self) -> usize {
        let mut result = self.nodes.len();
        if result != 0 { result -= 1; }
        result
    }

    pub fn clear(&mut self) {
        self.nodes.clear();
    }
}

pub fn orientation(p: Path) -> bool {
    area(p) >= 0.0
}

pub fn area(p: Path) -> f64 {

    let size = p.len();
    if size < 3 { return 0.0; };

    let mut a = 0f64;
    let mut j = size - 1;

    for i in 0..size {
        let pi_w = p.get(i);
        let pj_w = p.get(j);
        match (pi_w, pj_w) {
            (Some(pi), Some(pj)) => a += (pj.get_x()+pi.get_x())*(pj.get_y()-pi.get_y()),
            _ => ()
        }
        j = i;
    }

    -a as f64 * 0.5
}


/*
pub struct IntRect {
    pub left: isize,
    pub top: isize,
    pub right: isize,
    pub bottom: isize,
}
*/

pub struct IntersectNode {
    pub edge_1: ActiveEdge,
    pub edge_2: ActiveEdge,
    pub pt: DoublePoint,
}

#[derive(PartialEq, Copy, Clone)]
pub enum VertexFlag {
    None = 0,
    OpenStart = 1,
    OpenEnd = 2,
    LocalMax = 4,
    LocalMin = 8
}

pub struct LocalMinimum {
    #[cfg(use_int32)]
    y: isize,
    #[cfg(not(use_int32))]
    y: u64,
    left_bound: ActiveEdge,
    right_bound: ActiveEdge,
    _type: PhantomData<DoublePoint>,
}

#[derive(PartialEq)]
pub struct OutPt {
    pub idx: usize,
    pub pt: DoublePoint,
    pub next: Arc<OutPt>,
    pub prev: Arc<OutPt>,
}

impl OutPt {
    // TODO!!
    pub fn area(&self) -> f64 {
        let start = self.next.clone();
        let mut area = 0.0;
        let mut op = start.clone();
        loop {
            area += ((op.prev.pt.get_x() + op.pt.get_x()) *
                     (op.prev.pt.get_y() - op.pt.get_y())) as f64;
            op = op.next.clone();
            if *op == *start { break; }
        }

        area * 0.5
    }

    /*
    pub fn reverse_poly_pt_list(&mut self) {

        // not possible in the rust model, also very bad for cache
        let start = self.next.clone();
        let mut op = start.clone();
        loop {
            let pp2 = op.next.clone();
            op.next = op.prev.clone();
            op.prev = pp2.clone();
            op = pp2;
            if *op == *start { break; }
        }
            if (!pp) return;
            OutPt *pp1, *pp2;
            pp1 = pp;
            do {
            pp2 = pp1->Next;
            pp1->Next = pp1->Prev;
            pp1->Prev = pp2;
            pp1 = pp2;
            } while( pp1 != pp );
    }
    */
}


pub struct OutRec {
    pub idx: usize,
    // TODO: is_hole and is_open can be merged!!!
    pub is_hole_open: u8,
    //see comments in clipper.pas
    pub first_left: Arc<OutRec>,
    pub poly_node: Arc<PolyNode>,
    pub pts: Arc<OutPt>,
    pub bottom_pt: Arc<OutPt>,
}

impl OutRec {
    pub fn area(&self) -> f64 {
        self.pts.area()
    }
}


impl OutRec {
    #[inline(always)]
    pub fn is_hole(&self) -> bool {
        self.is_hole_open & IS_HOLE == 0
    }
    #[inline(always)]
    pub fn is_open(&self) -> bool {
        self.is_hole_open & IS_OPEN == 0
    }
}

pub struct Join {
    pub out_pt1: Arc<OutPt>,
    pub out_pt2: Arc<OutPt>,
    pub off_pt: DoublePoint,
}

pub fn point_is_vertex(pt: &DoublePoint, pp: Arc<OutPt>) -> bool {
    let mut pp2 = pp.clone();
    loop {
        if pp2.pt == *pt { return true; }
        pp2 = pp2.next.clone();
        if *pp2 == *pp { break; }
    }
    false
}

pub fn calculate_local_minima(path: &Path) -> Result<LocalMinima, ClipErr> {
    let local_minima = LocalMinima::new();
    Ok(local_minima)
}

pub fn validate_path(path: &Path) -> Result<Path, ClipErr> {
    let validated_path = Path::new();
    Ok(validated_path)
}

/*
pub fn path_to_vertices(path: &Path) -> Result<LinkedList<Vertex>, ClipErr> {
    let mut linked_list: LinkedList<Vertex> = LinkedList::new();
    for p in path.iter() {
        let vertex = Vertex{pt: *p, flags: VertexFlags::None};
        if linked_list.front() != Some(&vertex) {
            linked_list.push_back(vertex)
        }
    }
    Ok(linked_list)
}
 */

pub fn mark_sbt(){}
pub fn add_local_min(){}
pub fn add_local_max(){}
pub fn add_left(){}
pub fn add_right(){}
pub fn add_edges(){}
pub fn append_polygon(){}


pub fn clip_polys(subject: &Path, clip: &Path,  ) -> Result<Path, ClipErr>{
    // BuildSBT

    // LoopAEsDoHorizontal

    // HorzTrialsToJoins

    // DoIntersections

    // DoTopOfScanbeam

    // LoopAEsDoHorizontal

    /*
    InsertLocalMinimaIntoAEL(y);
    Active *e;
    while (PopHorz(e)) DoHorizontal(*e);
    if (horz_joiners_) ConvertHorzTrialsToJoins();
    bot_y_ = y;  //bot_y_ == bottom of scanbeam
    if (!PopScanline(y)) break;  //y new top of scanbeam
    DoIntersections(y);
    DoTopOfScanbeam(y);
    while (PopHorz(e)) DoHorizontal(*e);
     */
    /*
    Build LMT;
    Set AET to null;
    For each scanbeam do;
        Set yb and yt to bottom and top of the scanbeam.
        If LMT node corresponding to yb exists ;
            AddEdges(LMTnode)
        Build Intersection Table (IT) for the current scanbeam;
        For each node in IT;
            Set edge1 and edge2 from the
            IT node; /* edge1 precedes
            edge2 in AET */
            Classify the point of
            intersection 'p';
            Switch (class of p) do;
                Case(LI KE__EDGE_INT):
                    If edge 1 is contributing:
                        AddLeft(edge 1,p);
                        AddRight(edge2,p);
                        Exchange edge 1->side
                        and edge2->side;
                Case(LOCAL_MAX):
                    AddLocalMax(edge 1 ,p);
                Case(LEFT_INT):
                    AddLeft(edge2,p);
                Case(RIGHT_INT):
                    AddRight(edge I ,p);
                Case(LOCAL_MIN):
                    AddLocalMin(edge 1,p);
                Swap edge1 and edge2 positions in the AET;
                Exchange edge1->poly and edge2->poly;
        For each AETedge do;
            If AETedge is terminating at yt
                Classify the upper end vertex 'p' of AETedge;
                Switch (class of p)
                    Case(LOCAL_MAX):
                        AddLocalMax(AETedge,p);
                        Delete AETedge and AETedge->next from the AET;
                    Case(LEFT_INT):
                        AddLeft(edge2,p);
                        Replace AETedge by AETedge-> succ;
                    Case(RIGHT_INT):
                        AddRight(edge 1 ,p);
                        Replace AETedge by AETedge->succ;
     */
    let mut my_map:HashMap<OrderedFloat<f64>, bool> = HashMap::new();
    my_map.insert(OrderedFloat(1.0f64),false);

   Err(ClipErr)
}

//pub fn local_minima_to_



pub fn path_to_edges(path: &Path, fill_type: &PolyFillType) -> Result<Edges, ClipErr> {
    let poly: Vec<DoublePoint> = Vec::new();
    Ok(Edges::new())
}

pub fn paths_to_edges(paths: Paths, fill_type: &PolyFillType) -> Result<Vec<Edges>, ClipErr> {
    paths.iter().map(|p| path_to_edges(p, fill_type)).collect()
}

pub fn path_union(path1: &Path, path2: &Path, fill_type: &PolyFillType) -> Path {
    let poly: Vec<DoublePoint> = Vec::new();
    poly
}

/// See http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.88.5498&rep=rep1&type=pdf
/// returns 0 if false, +1 if true, -1 if pt ON polygon boundary
pub fn is_point_in_path<T: Coords<f64>>(pt: &T, path: &Path) -> i8 {

    if path.len() < 3 { return 0; }

    let mut result: i8 = 0;
    let ip_iter = path.iter();
    let mut np_iter = path.iter().cycle();
    np_iter.next();

    let pt_y = pt.get_y();
    let pt_x = pt.get_x();

    for (ip, np) in ip_iter.zip(np_iter) {

        let ip_x = ip.get_x();
        let ip_y = ip.get_y();
        let np_x = np.get_x();
        let np_y = np.get_y();

        if np_y == pt_y &&
           (np_x == pt_x || ip_y == np_y &&
           ((np_x > pt_x) == (ip_x < pt_x))) {
           return -1;
        }

        if (ip_y < pt_y) == (np_y < pt_y) { continue; }

        let cond1 = ip_x >= pt_x;

        if cond1 && (np_x > pt_x) {
            result = 1 - result;
            continue;
        }

        if cond1 || (np_x > pt_x) {

            let vec_a = ip_x - pt_x;
            let vec_b = np_y - pt_y;
            let vec_c = np_x - pt_x;
            let vec_d = ip_y - pt_y;

            let cond2 = np_y > ip_y;

            let d = vec_a * vec_b - vec_c * vec_d;
            if d == 0.0 {
                return -1;
            } else if (d > 0.0) == cond2 {
                result = 1 - result;
            }
        }
    }

    return result;
}

/// Checks if a point falls in an OutPt
/// renamed from `int PointInPolygon (const IntPoint &pt, OutPt *op)`
pub fn is_point_in_out_pt(pt: &DoublePoint, op: Arc<OutPt>) -> i8 {

    // This is different from the original algorithm:
    // Instead of following pointers, we collect the OutPt into a path
    // This provides better cache access + lets us reuse the point
    let mut out_path = Path::new();
    let origin_op = op.clone();
    let mut cur_op = op.clone();

    while cur_op != origin_op {
        out_path.push(cur_op.pt);
        cur_op = cur_op.next.clone();
    }

    is_point_in_path(pt, &out_path )
}

/// TODO: this works, but it is worst-case O(n^2)
/// as we check every point against every other point
///
/// In theory, this should perform better than the C++ version ("Poly2ContainsPoly1")
/// due to better cache access.
pub fn poly2_contains_poly1(pt1: Arc<OutPt>, pt2: Arc<OutPt>) -> bool {

    // create path for pt2
    let mut out_path = Path::new();
    let origin_op = pt2.clone();
    let mut cur_op = pt2.clone();

    while cur_op != origin_op {
        out_path.push(cur_op.pt);
        cur_op = cur_op.next.clone();
    }

    let pt2_path = out_path;

    let origin_op = pt1.clone();
    let mut cur_op = pt1.clone();

    while cur_op != origin_op {
        let res = is_point_in_path(&cur_op.pt, &pt2_path);
        if res >= 0 { return res > 0 }
        cur_op = cur_op.next.clone();
    }

    true
}


