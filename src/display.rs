use core::fmt::Write;
use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::Rgb565,
    prelude::*,
    primitives::*,
    text::{Alignment, Text},
};
use heapless::{Deque, String};
use profont;

pub struct Display {
    pub temperature: Deque<(i32, i32), 180>,
    pub humidity: Deque<(i32, i32), 180>,
    pub pressure: Deque<(i32, i32), 180>,
    pub co2: Deque<(i32, i32), 180>,
}

impl Display {
    pub fn new() -> Self {
        Display {
            temperature: Deque::<(i32, i32), 180>::new(),
            humidity: Deque::<(i32, i32), 180>::new(),
            pressure: Deque::<(i32, i32), 180>::new(),
            co2: Deque::<(i32, i32), 180>::new(),
        }
    }

    fn draw_grid<D, T>(&self, target: &mut D, top: i32)
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        let erase_style = PrimitiveStyle::with_fill(Rgb565::BLACK);
        let style = PrimitiveStyle::with_stroke(Rgb565::BLUE, 1);
        Rectangle::new(Point::new(0, top), Size::new(180, 60))
            .into_styled(erase_style)
            .draw(target)
            .unwrap();
        Rectangle::new(Point::new(0, top), Size::new(180, 60))
            .into_styled(style)
            .draw(target)
            .unwrap();

        // horizontal
        Line::new(Point::new(0, top + 30), Point::new(180, top + 30))
            .into_styled(style)
            .draw(target)
            .unwrap();
        // vertical
        for x in [45, 90, 135] {
            Line::new(Point::new(x, top), Point::new(x, top + 60))
                .into_styled(style)
                .draw(target)
                .unwrap();
        }
    }

    fn draw_graph<D, T>(
        &self,
        target: &mut D,
        top: i32,
        data: &Deque<(i32, i32), 180>,
        color: Rgb565,
    ) where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_grid(target, top);

        let style = PrimitiveStyle::with_stroke(color, 1);
        let mut x = (180 - data.len()) as i32;
        for v in data.iter() {
            let mut v1 = v.1;
            if v1 == v.0 {
                v1 = v1 + 1;
            }
            Line::new(Point::new(x, top + 60 - v.0), Point::new(x, top + 60 - v1))
                .into_styled(style)
                .draw(target)
                .unwrap();
            x = x + 1;
        }
    }

    pub fn draw_graph_temperature<D, T>(&self, target: &mut D, top: i32, color: Rgb565)
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_graph(target, top, &self.temperature, color);
    }

    pub fn draw_graph_humidity<D, T>(&self, target: &mut D, top: i32, color: Rgb565)
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_graph(target, top, &self.humidity, color);
    }

    pub fn draw_graph_pressure<D, T>(&self, target: &mut D, top: i32, color: Rgb565)
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_graph(target, top, &self.pressure, color);
    }

    pub fn draw_graph_co2<D, T>(&self, target: &mut D, top: i32, color: Rgb565)
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_graph(target, top, &self.co2, color);
    }

    fn draw_data<D, T>(&self, target: &mut D, top: i32, data_str: &str, range_str: &str)
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        let erase_style = PrimitiveStyle::with_fill(Rgb565::BLACK);
        let cur_style = MonoTextStyle::new(&profont::PROFONT_24_POINT, Rgb565::WHITE);
        let range_style = MonoTextStyle::new(&profont::PROFONT_14_POINT, Rgb565::YELLOW);

        Rectangle::new(Point::new(180, top), Size::new(140, 60))
            .into_styled(erase_style)
            .draw(target)
            .unwrap();

        Text::with_alignment(
            data_str,
            Point::new(190, top + 30),
            cur_style,
            Alignment::Left,
        )
        .draw(target)
        .unwrap();
        Text::with_alignment(
            range_str,
            Point::new(190, top + 50),
            range_style,
            Alignment::Left,
        )
        .draw(target)
        .unwrap();
    }

    fn draw_data_float<D, T>(
        &self,
        target: &mut D,
        top: i32,
        data: &f32,
        range: &(f32, f32),
        unit: &str,
    ) where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        let mut data_buf = String::<64>::new();
        let mut range_buf = String::<64>::new();

        data_buf.clear();
        write!(data_buf, "{:>2.1}{}", data, unit).unwrap();
        range_buf.clear();
        write!(range_buf, "{:>2.1} / {:>2.1}", range.1, range.0).unwrap();

        self.draw_data(target, top, &data_buf.as_str(), &range_buf.as_str());
    }

    fn draw_data_integer<D, T>(
        &self,
        target: &mut D,
        top: i32,
        data: &f32,
        range: &(f32, f32),
        unit: &str,
    ) where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        let mut data_buf = String::<64>::new();
        let mut range_buf = String::<64>::new();

        data_buf.clear();
        write!(data_buf, "{:>4.0}{}", data, unit).unwrap();
        range_buf.clear();
        write!(range_buf, "{:>4.0} / {:>4.0}", range.1, range.0).unwrap();

        self.draw_data(target, top, &data_buf.as_str(), &range_buf.as_str());
    }

    pub fn draw_data_temperature<D, T>(
        &self,
        target: &mut D,
        top: i32,
        data: &f32,
        range: &(f32, f32),
    ) where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_data_float(target, top, data, range, " C");
    }

    pub fn draw_data_humidity<D, T>(&self, target: &mut D, top: i32, data: &f32, range: &(f32, f32))
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_data_float(target, top, data, range, " %");
    }

    pub fn draw_data_pressure<D, T>(&self, target: &mut D, top: i32, data: &f32, range: &(f32, f32))
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_data_integer(target, top, data, range, "hPa");
    }

    pub fn draw_data_co2<D, T>(&self, target: &mut D, top: i32, data: &f32, range: &(f32, f32))
    where
        D: DrawTarget<Color = Rgb565, Error = T>,
        T: core::fmt::Debug,
    {
        self.draw_data_integer(target, top, data, range, "ppm");
    }
}
