use std::collections::HashMap;
/// Utility class implementing a simple one-off registry
pub struct Registry<T> {
    pub registry: HashMap<usize, T>,
    counter: usize,
}

impl<T> Registry<T> {
    pub fn new() -> Self {
        Registry {
            registry: HashMap::new(),
            counter: 0,
        }
    }

    pub fn add_new_item(&mut self, item: T) -> usize {
        let id = self.counter;
        self.registry.insert(id, item);
        self.counter = self.counter + 1;
        id
    }
}
