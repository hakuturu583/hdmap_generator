//! Insertion-ordered storage keyed by a typed identifier.

use std::collections::HashMap;
use std::hash::Hash;

/// A collection of IR objects addressed by their identifier.
///
/// Iteration is in insertion order, never in hash order, because the exporters walk
/// these to assign format-specific ids: a map built the same way twice has to export
/// the same way twice.
#[derive(Debug, Clone)]
pub struct Arena<K, V> {
    items: Vec<V>,
    index: HashMap<K, usize>,
}

/// Two arenas are equal when they hold the same values in the same order. The
/// lookup table is derived from that, so comparing it as well would only make the
/// comparison require `K: Eq + Hash` for no extra information.
impl<K, V: PartialEq> PartialEq for Arena<K, V> {
    fn eq(&self, other: &Self) -> bool {
        self.items == other.items
    }
}

impl<K, V> Default for Arena<K, V> {
    fn default() -> Self {
        Arena {
            items: Vec::new(),
            index: HashMap::new(),
        }
    }
}

/// Returned when an identifier is inserted twice.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DuplicateId<K>(pub K);

impl<K: Eq + Hash + Clone, V> Arena<K, V> {
    pub fn new() -> Self {
        Arena::default()
    }

    pub fn insert(&mut self, key: K, value: V) -> Result<(), DuplicateId<K>> {
        if self.index.contains_key(&key) {
            return Err(DuplicateId(key));
        }
        self.index.insert(key, self.items.len());
        self.items.push(value);
        Ok(())
    }

    pub fn get(&self, key: &K) -> Option<&V> {
        self.index.get(key).map(|&i| &self.items[i])
    }

    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        let index = *self.index.get(key)?;
        self.items.get_mut(index)
    }

    pub fn contains(&self, key: &K) -> bool {
        self.index.contains_key(key)
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Values in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = &V> {
        self.items.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut V> {
        self.items.iter_mut()
    }
}

impl<'a, K: Eq + Hash + Clone, V> IntoIterator for &'a Arena<K, V> {
    type Item = &'a V;
    type IntoIter = std::slice::Iter<'a, V>;
    fn into_iter(self) -> Self::IntoIter {
        self.items.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::id::RoadId;

    #[test]
    fn iteration_follows_insertion_not_hashing() {
        let mut arena: Arena<RoadId, &str> = Arena::new();
        for name in ["zulu", "alpha", "mike"] {
            arena.insert(RoadId::new(name), name).unwrap();
        }
        assert_eq!(
            arena.iter().copied().collect::<Vec<_>>(),
            ["zulu", "alpha", "mike"]
        );
    }

    #[test]
    fn an_identifier_cannot_be_inserted_twice() {
        let mut arena: Arena<RoadId, u8> = Arena::new();
        arena.insert(RoadId::new("a"), 1).unwrap();
        assert_eq!(
            arena.insert(RoadId::new("a"), 2),
            Err(DuplicateId(RoadId::new("a")))
        );
    }
}
