use heapless::Deque;

pub struct Dataset {
    pub max: f32,
    pub min: f32,
    pub current: (f32, f32),
    pub queue: Deque<(f32, f32), 180>,
}

impl Dataset {
    pub fn new(max: f32, min: f32) -> Self {
        Dataset {
            max: max,
            min: min,
            current: (min, max),
            queue: Deque::<(f32, f32), 180>::new(),
        }
    }

    pub fn range(&self) -> (f32, f32) {
        if self.queue.is_empty() {
            return (self.min, self.max);
        }
        let mut high = self.min;
        let mut low = self.max;
        for value in self.queue.iter() {
            if high < value.0 {
                high = value.0;
            }
            if low > value.1 {
                low = value.1;
            }
        }
        if self.current.0 > high {
            high = self.current.0;
        }
        if self.current.1 < low {
            low = self.current.1;
        }
        return (high, low);
    }

    pub fn append(&mut self, value: f32) {
        let mut high = self.current.0;
        let mut low = self.current.1;
        if value > high {
            high = value;
        }
        if value < low {
            low = value;
        }
        self.current = (high, low);
    }

    pub fn next(&mut self) {
        if self.queue.is_full() {
            self.queue.pop_front();
        }
        self.queue.push_back(self.current).unwrap();
        self.current = (self.min, self.max);
    }

    pub fn export(&self, max: f32, min: f32, pixels: i32) -> Deque<(i32, i32), 180> {
        let mut result = Deque::<(i32, i32), 180>::new();
        let range = max - min;
        let pix = pixels as f32;
        for value in self.queue.iter() {
            let mut high = (value.0 - min) / range * pix;
            if high > pix {
                high = pix;
            }
            if high < 0.0 {
                high = 0.0;
            }
            let mut low = (value.1 - min) / range * pix;
            if low > pix {
                low = pix;
            }
            if low < 0.0 {
                low = 0.0;
            }
            result.push_back((high as i32, low as i32)).unwrap();
        }
        return result;
    }
}
