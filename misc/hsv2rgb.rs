fn hsv2rgb(h: f64, s: f64, v: f64) -> (f64, f64, f64) {
    let c = s * v;
    let x = c * (1.0 - ((h % 2.0) - 1.0).abs());

    let (r, g, b): (f64, f64, f64) = if h < 1.0 {
        (c, x, 0.0)
    } else if h < 2.0 {
        (x, c, 0.0)
    } else if h < 3.0 {
        (0.0, c, x)
    } else if h < 4.0 {
        (0.0, x, c)
    } else if h < 5.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };

    let m = v - c;
    (r + m, g + m, b + m)
}

fn f(h: f64) -> (f64, f64, f64) {
    // Hue で V を決める
    // すごい急勾配な関数で赤いときに値が大きくなるように
    let v = (1.0 / (0.86 + h).powf(10.0) + 0.02).min(1.0);
    // S は固定 (下げると NeoPixel ではほぼ白になっちゃう)
    let s = 1.0;
    hsv2rgb(h, s, v)
}

fn do_dump() {
    for x in (0..256).rev() {
        let h = x as f64 / 255.0 * 4.0;
        let (r, g, b) = f(h);

        println!(
            "{:#04x}_{:02x}_{:02x}_00,",
            (g.clamp(0.0, 1.0) * 255.0) as u8,
            (r.clamp(0.0, 1.0) * 255.0) as u8,
            (b.clamp(0.0, 1.0) * 255.0) as u8,
        );
    }
}

fn do_img() {
    println!("P3");
    println!("256 256");
    println!("255");

    for _ in 0..256 {
        for x in (0..256).rev() {
            let h = x as f64 / 255.0 * 4.0;
            let (r, g, b) = f(h);

            println!(
                "{} {} {}",
                (r.clamp(0.0, 1.0) * 255.0) as u8,
                (g.clamp(0.0, 1.0) * 255.0) as u8,
                (b.clamp(0.0, 1.0) * 255.0) as u8,
            );
        }
    }
}

fn main() {
    match std::env::args().skip(1).next() {
        Some(arg) if arg == "img" => do_img(),
        _ => do_dump(),
    }
}
