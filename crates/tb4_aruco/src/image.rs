#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GrayImage {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

impl GrayImage {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            data: vec![0; width * height],
        }
    }

    pub fn filled(width: usize, height: usize, value: u8) -> Self {
        Self {
            width,
            height,
            data: vec![value; width * height],
        }
    }

    pub fn from_mono8(width: usize, height: usize, data: &[u8]) -> Option<Self> {
        Self::from_mono8_step(width, height, width, data)
    }

    pub fn from_rgb8(width: usize, height: usize, data: &[u8]) -> Option<Self> {
        Self::from_rgb8_step(width, height, width * 3, data)
    }

    pub fn from_bgr8(width: usize, height: usize, data: &[u8]) -> Option<Self> {
        Self::from_bgr8_step(width, height, width * 3, data)
    }

    pub fn from_mono8_step(width: usize, height: usize, step: usize, data: &[u8]) -> Option<Self> {
        let mut img = Self::default();
        img.set_mono8(width, height, step, data).then_some(img)
    }

    pub fn from_rgb8_step(width: usize, height: usize, step: usize, data: &[u8]) -> Option<Self> {
        let mut img = Self::default();
        img.set_color(width, height, step, data, false)
            .then_some(img)
    }

    pub fn from_bgr8_step(width: usize, height: usize, step: usize, data: &[u8]) -> Option<Self> {
        let mut img = Self::default();
        img.set_color(width, height, step, data, true)
            .then_some(img)
    }

    // In-place variants reuse the buffer; they return false if `data` is too short.
    pub fn set_mono8(&mut self, width: usize, height: usize, step: usize, data: &[u8]) -> bool {
        if !Self::fits(width, height, step, 1, data.len()) {
            return false;
        }
        self.resize(width, height);
        for (y, row) in self
            .data
            .chunks_exact_mut(width.max(1))
            .enumerate()
            .take(height)
        {
            row.copy_from_slice(&data[y * step..y * step + width]);
        }
        true
    }

    pub fn set_color(
        &mut self,
        width: usize,
        height: usize,
        step: usize,
        data: &[u8],
        bgr: bool,
    ) -> bool {
        if !Self::fits(width, height, step, 3, data.len()) {
            return false;
        }
        self.resize(width, height);
        let (ri, bi) = if bgr { (2, 0) } else { (0, 2) };
        for (y, row) in self
            .data
            .chunks_exact_mut(width.max(1))
            .enumerate()
            .take(height)
        {
            let src = &data[y * step..y * step + width * 3];
            for (dst, px) in row.iter_mut().zip(src.chunks_exact(3)) {
                let v = 77 * px[ri] as u32 + 150 * px[1] as u32 + 29 * px[bi] as u32;
                *dst = ((v + 128) >> 8) as u8;
            }
        }
        true
    }

    #[inline]
    pub fn get(&self, x: usize, y: usize) -> u8 {
        self.data[y * self.width + x]
    }

    // Bilinear sample with pixel centres at integer coordinates, clamped at the border.
    #[inline]
    pub fn sample(&self, x: f64, y: f64) -> f64 {
        let maxx = (self.width - 1) as f64;
        let maxy = (self.height - 1) as f64;
        let x = x.clamp(0.0, maxx);
        let y = y.clamp(0.0, maxy);
        let x0 = x.floor() as usize;
        let y0 = y.floor() as usize;
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.height - 1);
        let fx = x - x0 as f64;
        let fy = y - y0 as f64;
        let row0 = y0 * self.width;
        let row1 = y1 * self.width;
        let a = self.data[row0 + x0] as f64;
        let b = self.data[row0 + x1] as f64;
        let c = self.data[row1 + x0] as f64;
        let d = self.data[row1 + x1] as f64;
        let top = a + (b - a) * fx;
        let bot = c + (d - c) * fx;
        top + (bot - top) * fy
    }

    fn fits(width: usize, height: usize, step: usize, ch: usize, len: usize) -> bool {
        step >= width * ch && (height == 0 || len >= (height - 1) * step + width * ch)
    }

    fn resize(&mut self, width: usize, height: usize) {
        self.width = width;
        self.height = height;
        self.data.resize(width * height, 0);
    }
}
