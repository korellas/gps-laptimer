#!/usr/bin/env python3
"""
TTF to LVGL C font converter
Converts TrueType fonts to LVGL format with 4bpp (16 grayscale levels)
"""

import freetype
import sys
from pathlib import Path


def pack_bitmap_to_4bpp(bitmap):
    """Pack FreeType 8bpp grayscale bitmap to LVGL 4bpp stream (no row alignment)."""
    pixels_4bpp = []

    if bitmap.width <= 0 or bitmap.rows <= 0:
        return pixels_4bpp

    row_stride = abs(bitmap.pitch)
    pending_nibble = None

    for y in range(bitmap.rows):
        # FreeType can expose negative pitch (bottom-up rows).
        src_row = y if bitmap.pitch >= 0 else (bitmap.rows - 1 - y)
        row_start = src_row * row_stride

        for x in range(bitmap.width):
            nibble = bitmap.buffer[row_start + x] >> 4
            if pending_nibble is None:
                pending_nibble = nibble
            else:
                pixels_4bpp.append((pending_nibble << 4) | nibble)
                pending_nibble = None

    if pending_nibble is not None:
        pixels_4bpp.append(pending_nibble << 4)

    return pixels_4bpp


def convert_ttf_to_lvgl(ttf_path, size, output_path, font_name, tracking=0):
    """Convert TTF font to LVGL C format. tracking: pixel adjustment to advance width (negative = tighter)"""

    # ASCII printable range
    char_range = range(0x20, 0x7F)  # 0x20-0x7E

    face = freetype.Face(str(ttf_path))
    face.set_pixel_sizes(0, size)

    glyphs = {}
    max_ascent = 0
    max_descent = 0

    # Render all glyphs
    for char_code in char_range:
        face.load_char(char_code, freetype.FT_LOAD_RENDER)
        bitmap = face.glyph.bitmap

        # Get metrics (apply tracking adjustment)
        advance = max(1, (face.glyph.advance.x >> 6) + tracking)
        bearing_x = face.glyph.bitmap_left
        bearing_y = face.glyph.bitmap_top

        # Convert bitmap to 4bpp
        pixels_4bpp = pack_bitmap_to_4bpp(bitmap)

        glyphs[char_code] = {
            'bitmap': pixels_4bpp,
            'width': bitmap.width,
            'height': bitmap.rows,
            'advance': advance,
            'bearing_x': bearing_x,
            'bearing_y': bearing_y
        }

        ascent = bearing_y
        descent = bitmap.rows - bearing_y
        max_ascent = max(max_ascent, ascent)
        max_descent = max(max_descent, descent)

    line_height = max_ascent + max_descent

    # Generate C file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(f"""/*******************************************************************************
 * Size: {size} px
 * Bpp: 4
 * Opts: --font {ttf_path.name} --range 0x20-0x7E --size {size} --bpp 4
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifndef {font_name.upper()}
#define {font_name.upper()} 1
#endif

#if {font_name.upper()}

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {{
""")

        # Write bitmap data
        offset = 0
        glyph_offsets = {}
        for char_code in char_range:
            glyph = glyphs[char_code]
            glyph_offsets[char_code] = offset

            if glyph['bitmap']:
                f.write(f"    /* U+{char_code:04X} */\n    ")
                for i, byte in enumerate(glyph['bitmap']):
                    f.write(f"0x{byte:02x}, ")
                    if (i + 1) % 16 == 0:
                        f.write("\n    ")
                f.write("\n\n")
                offset += len(glyph['bitmap'])

        f.write("};\n\n")

        # Write glyph descriptors
        f.write("""/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
""")

        # id = 0 is reserved in LVGL for missing glyph
        f.write("    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},\n")

        for char_code in char_range:
            glyph = glyphs[char_code]
            bitmap_index = glyph_offsets[char_code]
            # LVGL expects ofs_y relative to baseline:
            # glyph_top = baseline - box_h - ofs_y  =>  ofs_y = bitmap_top - box_h
            ofs_y = glyph['bearing_y'] - glyph['height']

            f.write(f"    {{.bitmap_index = {bitmap_index}, .adv_w = {glyph['advance'] * 16}, "
                   f".box_w = {glyph['width']}, .box_h = {glyph['height']}, "
                   f".ofs_x = {glyph['bearing_x']}, .ofs_y = {ofs_y}}},\n")

        f.write("};\n\n")

        # Write character map
        f.write("""/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

""")

        min_char = min(char_range)
        max_char = max(char_range)

        f.write(f"""/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] = {{
    {{
        .range_start = {min_char}, .range_length = {max_char - min_char + 1}, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }}
}};

""")

        # Write font descriptor
        f.write(f"""/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LV_VERSION_CHECK(8, 0, 0)
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {{
#else
static lv_font_fmt_txt_dsc_t font_dsc = {{
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 4,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LV_VERSION_CHECK(8, 0, 0)
    .cache = &cache
#endif
}};

/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LV_VERSION_CHECK(8, 0, 0)
const lv_font_t {font_name} = {{
#else
lv_font_t {font_name} = {{
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = {line_height},          /*The maximum line height required by the font*/
    .base_line = {max_descent},             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = {-size // 10},
    .underline_thickness = {max(1, size // 20)},
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
}};

#endif /*#{font_name.upper()}*/
""")

    print(f"[OK] Generated: {output_path.name}")
    return True

if __name__ == '__main__':
    if len(sys.argv) < 5 or len(sys.argv) > 6:
        print("Usage: convert_font_to_lvgl.py <ttf_file> <size> <output.c> <font_name> [tracking]")
        print("  tracking: pixel adjustment to advance width (e.g. -1 for tighter spacing)")
        sys.exit(1)

    ttf_path = Path(sys.argv[1])
    size = int(sys.argv[2])
    output_path = Path(sys.argv[3])
    font_name = sys.argv[4]
    tracking = int(sys.argv[5]) if len(sys.argv) == 6 else 0

    if not ttf_path.exists():
        print(f"Error: {ttf_path} not found")
        sys.exit(1)

    convert_ttf_to_lvgl(ttf_path, size, output_path, font_name, tracking)
