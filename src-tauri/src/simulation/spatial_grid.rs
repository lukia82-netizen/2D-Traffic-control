use std::collections::HashMap;

/// Uniform spatial hash grid for fast neighbour lookups.
/// Cell size is in degrees (~50 m at 50°N latitude when set to ~0.00045°).
pub struct SpatialGrid {
    pub cell_size: f64,
    pub cells: HashMap<(i32, i32), Vec<u32>>,
}

impl SpatialGrid {
    pub fn new(cell_size: f64) -> Self {
        SpatialGrid {
            cell_size,
            cells: HashMap::new(),
        }
    }

    #[inline]
    fn cell_of(&self, lat: f64, lng: f64) -> (i32, i32) {
        (
            (lat / self.cell_size).floor() as i32,
            (lng / self.cell_size).floor() as i32,
        )
    }

    /// Remove all vehicles from the grid (called at the start of each tick).
    pub fn clear(&mut self) {
        for v in self.cells.values_mut() {
            v.clear();
        }
    }

    /// Insert a vehicle id at the given geographic position.
    pub fn insert(&mut self, id: u32, lat: f64, lng: f64) {
        let cell = self.cell_of(lat, lng);
        self.cells.entry(cell).or_default().push(id);
    }

    /// Return all vehicle ids within `radius_cells` cells of `(lat, lng)`.
    pub fn query_nearby(&self, lat: f64, lng: f64, radius_cells: i32) -> Vec<u32> {
        let (cy, cx) = self.cell_of(lat, lng);
        let mut result = Vec::new();

        for dy in -radius_cells..=radius_cells {
            for dx in -radius_cells..=radius_cells {
                let key = (cy + dy, cx + dx);
                if let Some(ids) = self.cells.get(&key) {
                    result.extend_from_slice(ids);
                }
            }
        }

        result
    }
}
