#!/usr/bin/env python3
"""Convert hiki-ava.png to C bitmap header for e-ink display.

Output format: MSB-first packed bits, 1=WHITE 0=BLACK.
Matches Paint_DrawImage() in GUI_Paint.cpp.

Dependencies: Pillow only (no numpy).
"""

import sys
from pathlib import Path
from PIL import Image, ImageEnhance, ImageOps

SRC = Path(__file__).parent.parent / "hiki-ava.png"
DST = Path(__file__).parent.parent / "src" / "hiki_bitmaps.h"

W, H = 100, 130


def crop_character(img: Image.Image) -> Image.Image:
    """Crop to the character area (remove blue background margins)."""
    w, h = img.size
    left = int(w * 0.15)
    right = int(w * 0.85)
    top = int(h * 0.10)
    bottom = int(h * 0.72)
    return img.crop((left, top, right, bottom))


def to_monochrome(img: Image.Image, threshold: int = 100) -> Image.Image:
    """Convert to 1-bit monochrome for e-ink.

    The image is blue-dominant. We extract per-pixel luminance weighting
    R and G higher to pull out the character from the blue background.
    """
    rgb = img.convert("RGB")
    w, h = rgb.size
    pixels = rgb.load()

    # Build grayscale image with custom channel weights
    gray = Image.new("L", (w, h))
    gray_pixels = gray.load()

    min_val = 255.0
    max_val = 0.0

    # First pass: compute range
    for y in range(h):
        for x in range(w):
            r, g, b = pixels[x, y]
            val = 0.4 * r + 0.4 * g + 0.2 * b
            if val < min_val:
                min_val = val
            if val > max_val:
                max_val = val

    # Second pass: normalize and write
    span = max_val - min_val if max_val > min_val else 1.0
    for y in range(h):
        for x in range(w):
            r, g, b = pixels[x, y]
            val = 0.4 * r + 0.4 * g + 0.2 * b
            normalized = int((val - min_val) / span * 255)
            gray_pixels[x, y] = normalized

    # Boost contrast
    gray = ImageEnhance.Contrast(gray).enhance(2.5)

    # Apply threshold
    return gray.point(lambda x: 255 if x > threshold else 0, mode="1")


def to_monochrome_dithered(img: Image.Image) -> Image.Image:
    """Floyd-Steinberg dithering variant for more detail."""
    rgb = img.convert("RGB")
    w, h = rgb.size
    pixels = rgb.load()

    gray = Image.new("L", (w, h))
    gray_pixels = gray.load()

    min_val = 255.0
    max_val = 0.0
    for y in range(h):
        for x in range(w):
            r, g, b = pixels[x, y]
            val = 0.4 * r + 0.4 * g + 0.2 * b
            if val < min_val:
                min_val = val
            if val > max_val:
                max_val = val

    span = max_val - min_val if max_val > min_val else 1.0
    for y in range(h):
        for x in range(w):
            r, g, b = pixels[x, y]
            val = 0.4 * r + 0.4 * g + 0.2 * b
            gray_pixels[x, y] = int((val - min_val) / span * 255)

    gray = ImageEnhance.Contrast(gray).enhance(2.0)

    # Floyd-Steinberg dithering (Pillow default L → 1)
    return gray.convert("1")


def bitmap_to_c_array(bmp: Image.Image, name: str) -> str:
    """Convert 1-bit PIL image to C PROGMEM array.

    Format: MSB-first packed bytes. bit=1 → WHITE, bit=0 → BLACK.
    """
    w, h = bmp.size
    bytes_per_row = (w + 7) // 8
    data = []

    for y in range(h):
        for bx in range(bytes_per_row):
            byte_val = 0
            for bit in range(8):
                x = bx * 8 + bit
                if x < w:
                    pixel = bmp.getpixel((x, y))
                    if pixel:  # white pixel → bit=1
                        byte_val |= (0x80 >> bit)
            data.append(byte_val)

    lines = [f"// {name}: {w}x{h}, {len(data)} bytes"]
    lines.append(f"static const unsigned char {name}[] PROGMEM = {{")
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        hex_vals = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(f"    {hex_vals},")
    lines.append("};")
    return "\n".join(lines)


def generate_worried(normal_bmp: Image.Image) -> Image.Image:
    """Generate 'worried' variant with small visual differences."""
    w, h = normal_bmp.size
    worried = normal_bmp.copy().convert("L")
    px = worried.load()

    # Add dashed "worry lines" near top (forehead area)
    for y_off in [int(h * 0.12), int(h * 0.15)]:
        x_start = int(w * 0.3)
        x_end = int(w * 0.7)
        if y_off < h:
            for x in range(x_start, min(x_end, w)):
                if x % 3 != 0:
                    px[x, y_off] = 0

    return worried.convert("1")


def main():
    method = sys.argv[1] if len(sys.argv) > 1 else "threshold"
    threshold = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    print(f"Loading {SRC}...")
    img = Image.open(SRC)
    print(f"  Original: {img.size}, mode={img.mode}")

    cropped = crop_character(img)
    print(f"  Cropped: {cropped.size}")

    # Resize maintaining aspect ratio
    cropped.thumbnail((W, H), Image.LANCZOS)
    # Center on white canvas
    canvas = Image.new("RGBA", (W, H), (0, 0, 255, 255))
    x_off = (W - cropped.size[0]) // 2
    y_off = (H - cropped.size[1]) // 2
    canvas.paste(cropped, (x_off, y_off))
    print(f"  Canvas: {W}x{H}")

    if method == "dither":
        mono = to_monochrome_dithered(canvas)
        print("  Method: Floyd-Steinberg dithering")
    else:
        mono = to_monochrome(canvas, threshold)
        print(f"  Method: threshold={threshold}")

    preview_path = SRC.parent / "tools" / "preview_normal.png"
    mono.save(preview_path)
    print(f"  Preview: {preview_path}")

    worried = generate_worried(mono)
    worried_path = SRC.parent / "tools" / "preview_worried.png"
    worried.save(worried_path)
    print(f"  Worried preview: {worried_path}")

    normal_c = bitmap_to_c_array(mono, "hiki_normal")
    worried_c = bitmap_to_c_array(worried, "hiki_worried")

    header = f"""#pragma once
// Auto-generated by tools/png_to_bitmap.py from hiki-ava.png
// Mascot bitmaps for e-ink display (400x300, monochrome)
// Format: MSB-first, 1=WHITE 0=BLACK (matches Paint_DrawImage)

#define MASCOT_W {W}
#define MASCOT_H {H}

{normal_c}

{worried_c}
"""
    DST.write_text(header)
    print(f"  Header: {DST}")
    bytes_per = (W + 7) // 8 * H
    print(f"  Size per bitmap: {bytes_per} bytes")
    print("Done!")


if __name__ == "__main__":
    main()
