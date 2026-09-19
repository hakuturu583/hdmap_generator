//! A uniform grid of buckets over the plane, for "what is near this point?".
//!
//! Every spatial question the generators ask — which road outlines could this
//! ground vertex be inside, which buildings could this one overlap, which road
//! edge is within a verge of here — is asked once per candidate against everything
//! on the map, and the product is big enough to notice. A grid is the whole
//! answer: each thing is filed under every cell its box touches, and a query reads
//! the few cells round it. There is no tree here and no need for one.

use std::collections::HashMap;

/// Things filed under the cells of a uniform grid, by the box round each.
#[derive(Debug, Clone)]
pub struct Grid<T> {
    cell: f64,
    buckets: HashMap<(i64, i64), Vec<T>>,
}

impl<T> Grid<T> {
    /// An empty grid whose cells are `cell` metres across, and never less than one.
    pub fn new(cell: f64) -> Grid<T> {
        Grid {
            cell: cell.max(1.0),
            buckets: HashMap::new(),
        }
    }

    /// How wide the cells are.
    pub fn cell(&self) -> f64 {
        self.cell
    }

    /// Files `item` under every cell the box from `min` to `max` touches. A box
    /// that is not finite is filed nowhere.
    pub fn insert(&mut self, min: [f64; 2], max: [f64; 2], item: T)
    where
        T: Clone,
    {
        if !(min[0].is_finite() && min[1].is_finite() && max[0].is_finite() && max[1].is_finite()) {
            return;
        }
        let (x0, x1) = (self.index(min[0]), self.index(max[0]));
        let (y0, y1) = (self.index(min[1]), self.index(max[1]));
        for i in x0..=x1 {
            for j in y0..=y1 {
                self.buckets.entry((i, j)).or_default().push(item.clone());
            }
        }
    }

    fn index(&self, value: f64) -> i64 {
        (value / self.cell).floor() as i64
    }

    /// The items filed under the point's own cell.
    pub fn at(&self, x: f64, y: f64) -> impl Iterator<Item = &T> {
        self.buckets
            .get(&(self.index(x), self.index(y)))
            .into_iter()
            .flatten()
    }

    /// The items filed under the point's cell and the eight round it: everything
    /// within a cell's width of the point, and some things a little further.
    pub fn around(&self, x: f64, y: f64) -> impl Iterator<Item = &T> {
        let (i, j) = (self.index(x), self.index(y));
        (i - 1..=i + 1).flat_map(move |i| {
            (j - 1..=j + 1).flat_map(move |j| self.buckets.get(&(i, j)).into_iter().flatten())
        })
    }

    /// The items filed under every cell the box from `min` to `max` touches. An
    /// item filed under several of those cells comes out once per cell.
    pub fn covering(&self, min: [f64; 2], max: [f64; 2]) -> impl Iterator<Item = &T> {
        let (x0, x1) = (self.index(min[0]), self.index(max[0]));
        let (y0, y1) = (self.index(min[1]), self.index(max[1]));
        (x0..=x1).flat_map(move |i| {
            (y0..=y1).flat_map(move |j| self.buckets.get(&(i, j)).into_iter().flatten())
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_item_is_found_from_every_cell_its_box_touches_and_no_other() {
        let mut grid = Grid::new(10.0);
        grid.insert([5.0, 5.0], [15.0, 5.0], "road");
        assert_eq!(grid.at(7.0, 7.0).count(), 1);
        assert_eq!(grid.at(12.0, 7.0).count(), 1);
        assert_eq!(grid.at(27.0, 7.0).count(), 0);
        // Around: the cell at x = 22 is beside the one at 12.
        assert_eq!(grid.around(22.0, 7.0).count(), 1);
        assert_eq!(grid.around(37.0, 7.0).count(), 0);
        // Covering: a box across both cells sees the item twice, once per cell.
        assert_eq!(grid.covering([0.0, 0.0], [19.0, 9.0]).count(), 2);
    }

    #[test]
    fn a_box_that_is_not_finite_is_filed_nowhere() {
        let mut grid = Grid::new(10.0);
        grid.insert([f64::INFINITY, 0.0], [f64::NEG_INFINITY, 0.0], ());
        assert_eq!(grid.at(0.0, 0.0).count(), 0);
    }
}
