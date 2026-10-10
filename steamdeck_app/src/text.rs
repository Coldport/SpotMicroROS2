//! Anti-aliased text: DejaVu Sans (regular + bold, embedded in the binary) is
//! rasterized with fontdue into one glyph atlas per size at startup, so drawing
//! text is just texture copies — and the Deck needs no SDL2_ttf.

use std::collections::HashMap;

use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::video::{Window, WindowContext};

static REGULAR: &[u8] = include_bytes!("../assets/DejaVuSans.ttf");
static BOLD: &[u8] = include_bytes!("../assets/DejaVuSans-Bold.ttf");

/// Pixel sizes the UI uses; anything else snaps to the nearest one.
pub const SIZES: [u32; 7] = [12, 14, 16, 18, 22, 28, 40];
/// Non-ASCII characters the UI draws.
const EXTRA: &str = "°±·•×—–→←↑↓↕↔▲▼◀▶✓✕☰…‹›";

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum Weight {
    Regular,
    Bold,
}

#[derive(Copy, Clone)]
struct Glyph {
    src: Rect,
    /// offset from the pen position to the bitmap's top-left (y relative to the baseline)
    dx: i32,
    dy: i32,
    advance: f32,
}

struct Atlas {
    tex: Texture,
    glyphs: HashMap<char, Glyph>,
    ascent: i32,
}

pub struct Fonts {
    atlases: HashMap<(Weight, u32), Atlas>,
}

fn charset() -> Vec<char> {
    (32u8..127).map(|b| b as char).chain(EXTRA.chars()).collect()
}

fn build_atlas(font: &fontdue::Font, px: u32, tc: &TextureCreator<WindowContext>) -> Result<Atlas, String> {
    let chars = charset();
    let rendered: Vec<(char, fontdue::Metrics, Vec<u8>)> = chars
        .iter()
        .map(|&c| {
            let (m, bmp) = font.rasterize(c, px as f32);
            (c, m, bmp)
        })
        .collect();

    // Shelf-pack into a fixed-width atlas.
    let width = 1024u32;
    let (mut x, mut y, mut row_h) = (1u32, 1u32, 0u32);
    let mut places = Vec::with_capacity(rendered.len());
    for (_, m, _) in &rendered {
        let (w, h) = (m.width as u32, m.height as u32);
        if x + w + 1 > width {
            x = 1;
            y += row_h + 1;
            row_h = 0;
        }
        places.push((x, y));
        x += w + 1;
        row_h = row_h.max(h);
    }
    let height = (y + row_h + 1).next_power_of_two().max(8);

    let mut rgba = vec![0u8; (width * height * 4) as usize];
    let mut glyphs = HashMap::new();
    for ((c, m, bmp), (gx, gy)) in rendered.iter().zip(places) {
        // Glyph missing from the font: fontdue draws .notdef; skip it so it falls back to '?'.
        if font.lookup_glyph_index(*c) == 0 && *c != ' ' {
            continue;
        }
        for row in 0..m.height {
            for col in 0..m.width {
                let a = bmp[row * m.width + col];
                let o = (((gy as usize + row) * width as usize) + gx as usize + col) * 4;
                rgba[o..o + 4].copy_from_slice(&[255, 255, 255, a]);
            }
        }
        glyphs.insert(
            *c,
            Glyph {
                src: Rect::new(gx as i32, gy as i32, m.width.max(1) as u32, m.height.max(1) as u32),
                dx: m.xmin,
                dy: -(m.height as i32 + m.ymin),
                advance: m.advance_width,
            },
        );
    }
    let lm = font.horizontal_line_metrics(px as f32).ok_or("font has no horizontal metrics")?;

    // Linear filtering is read at texture creation; text is drawn 1:1 but this
    // keeps it smooth if the window is scaled.
    sdl2::hint::set("SDL_RENDER_SCALE_QUALITY", "linear");
    let created = tc.create_texture_static(PixelFormatEnum::RGBA32, width, height);
    sdl2::hint::set("SDL_RENDER_SCALE_QUALITY", "nearest");
    let mut tex = created.map_err(|e| e.to_string())?;
    tex.update(None, &rgba, (width * 4) as usize).map_err(|e| e.to_string())?;
    tex.set_blend_mode(BlendMode::Blend);
    Ok(Atlas { tex, glyphs, ascent: lm.ascent.round() as i32 })
}

impl Fonts {
    pub fn new(tc: &TextureCreator<WindowContext>) -> Result<Fonts, String> {
        let settings = fontdue::FontSettings::default();
        let regular = fontdue::Font::from_bytes(REGULAR, settings).map_err(|e| e.to_string())?;
        let bold = fontdue::Font::from_bytes(BOLD, fontdue::FontSettings::default()).map_err(|e| e.to_string())?;
        let mut atlases = HashMap::new();
        for &px in &SIZES {
            atlases.insert((Weight::Regular, px), build_atlas(&regular, px, tc)?);
            atlases.insert((Weight::Bold, px), build_atlas(&bold, px, tc)?);
        }
        Ok(Fonts { atlases })
    }

    fn atlas(&self, weight: Weight, px: u32) -> &Atlas {
        let size = *SIZES.iter().min_by_key(|s| (**s as i32 - px as i32).abs()).unwrap();
        &self.atlases[&(weight, size)]
    }

    fn glyph(a: &Atlas, c: char) -> Glyph {
        a.glyphs
            .get(&c)
            .or_else(|| a.glyphs.get(&'?'))
            .copied()
            .unwrap_or(Glyph { src: Rect::new(0, 0, 1, 1), dx: 0, dy: 0, advance: 0.0 })
    }

    pub fn width(&self, s: &str, weight: Weight, px: u32) -> i32 {
        let a = self.atlas(weight, px);
        s.chars().map(|c| Self::glyph(a, c).advance).sum::<f32>().round() as i32
    }

    /// Draw with the top of the line box at `y`. Returns the width drawn.
    pub fn draw(
        &mut self,
        canvas: &mut Canvas<Window>,
        s: &str,
        x: i32,
        y: i32,
        weight: Weight,
        px: u32,
        color: Color,
    ) -> i32 {
        let size = *SIZES.iter().min_by_key(|sz| (**sz as i32 - px as i32).abs()).unwrap();
        let a = self.atlases.get_mut(&(weight, size)).unwrap();
        a.tex.set_color_mod(color.r, color.g, color.b);
        a.tex.set_alpha_mod(color.a);
        let baseline = y + a.ascent;
        let mut pen = x as f32;
        for c in s.chars() {
            let g = Self::glyph(a, c);
            if g.src.width() > 1 || g.src.height() > 1 {
                let dst = Rect::new(pen.round() as i32 + g.dx, baseline + g.dy, g.src.width(), g.src.height());
                let _ = canvas.copy(&a.tex, g.src, dst);
            }
            pen += g.advance;
        }
        (pen - x as f32).round() as i32
    }

    /// Greedy word wrap to `max_w` pixels.
    pub fn wrap(&self, s: &str, weight: Weight, px: u32, max_w: i32) -> Vec<String> {
        let mut lines = Vec::new();
        let mut cur = String::new();
        for word in s.split_whitespace() {
            let candidate = if cur.is_empty() { word.to_string() } else { format!("{cur} {word}") };
            if self.width(&candidate, weight, px) > max_w && !cur.is_empty() {
                lines.push(std::mem::replace(&mut cur, word.to_string()));
            } else {
                cur = candidate;
            }
        }
        if !cur.is_empty() {
            lines.push(cur);
        }
        lines
    }

    /// Cut `s` down with an ellipsis so it fits in `max_w`.
    pub fn fit(&self, s: &str, weight: Weight, px: u32, max_w: i32) -> String {
        if self.width(s, weight, px) <= max_w {
            return s.to_string();
        }
        let mut out: String = s.to_string();
        while !out.is_empty() && self.width(&format!("{out}…"), weight, px) > max_w {
            out.pop();
        }
        format!("{}…", out.trim_end())
    }
}
