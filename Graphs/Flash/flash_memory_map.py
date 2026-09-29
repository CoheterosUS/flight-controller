"""Generate an SVG map of the W25Q32JV flash layout used by the flight controller."""

from argparse import ArgumentParser
from html import escape
from pathlib import Path


TOTAL_SIZE = 4 * 1024 * 1024
SECTOR_SIZE = 4096
PAGE_SIZE = 256
RECORD_SIZE = 32
RECORDS_PER_PAGE = 8
HEADER_SIZE = 12
DATA_START = SECTOR_SIZE


def fmt_address(value: int) -> str:
    return f"0x{value:06X}"


def rect(x: float, y: float, width: float, height: float, fill: str) -> str:
    return f'<rect x="{x:.1f}" y="{y:.1f}" width="{width:.1f}" height="{height:.1f}" fill="{fill}" stroke="#263238"/>'


def text(x: float, y: float, value: str, size: int = 16, weight: str = "normal") -> str:
    return f'<text x="{x:.1f}" y="{y:.1f}" font-family="Arial, sans-serif" font-size="{size}px" font-weight="{weight}" fill="#17202A">{escape(value)}</text>'


def make_svg(write_pointer: int, flight_count: int) -> str:
    write_pointer = max(DATA_START, min(TOTAL_SIZE, write_pointer))
    data_capacity = TOTAL_SIZE - DATA_START
    data_used = write_pointer - DATA_START
    data_free = data_capacity - data_used
    data_pages = data_capacity // PAGE_SIZE
    data_sectors = data_capacity // SECTOR_SIZE
    used_pages = data_used // PAGE_SIZE
    max_records = data_pages * RECORDS_PER_PAGE

    width, height = 1300, 840
    bar_x, bar_y, bar_w, bar_h = 80, 175, 1140, 76
    header_w = max(bar_w * HEADER_SIZE / TOTAL_SIZE, 18)
    data_bar_w = bar_w - header_w
    used_w = data_bar_w * data_used / data_capacity
    free_w = data_bar_w - used_w

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#F8FAFC"/>',
        text(80, 55, "W25Q32JV flash memory map", 30, "bold"),
        text(80, 88, "Flight-controller external flash: 4 MiB total", 18),
        text(80, 125, f"Assumed current write pointer: {fmt_address(write_pointer)}   |   flights: {flight_count}", 16),
        text(bar_x, 150, "Address", 14, "bold"),
        text(bar_x, 275, fmt_address(0), 14),
        text(bar_x + bar_w, 275, fmt_address(TOTAL_SIZE), 14),
        rect(bar_x, bar_y, header_w, bar_h, "#F4B942"),
        rect(bar_x + header_w, bar_y, used_w, bar_h, "#4C9F70"),
        rect(bar_x + header_w + used_w, bar_y, free_w, bar_h, "#D9E2EC"),
        text(bar_x + 8, bar_y + 31, "Header\n", 15, "bold"),
        text(bar_x + header_w + 12, bar_y + 31, "Used data", 15, "bold") if used_w >= 80 else "",
        text(bar_x + header_w + used_w + 12, bar_y + 31, "Free", 15, "bold") if free_w >= 80 else "",
        text(80, 330, "Fixed regions", 22, "bold"),
        text(100, 370, f"Header sector: {fmt_address(0)}–{fmt_address(DATA_START - 1)}  ({SECTOR_SIZE:,} B / 4 KiB)", 17),
        text(125, 400, f"FlashHeader_t: {HEADER_SIZE} B at {fmt_address(0)}", 16),
        text(125, 430, f"  Magic: 4 B   FlightCount: 4 B   WritePointer: 4 B", 16),
        text(125, 460, f"  Unused/reserved in header sector: {SECTOR_SIZE - HEADER_SIZE:,} B", 16),
        text(100, 505, f"Data region: {fmt_address(DATA_START)}–{fmt_address(TOTAL_SIZE - 1)}  ({data_capacity:,} B)", 17),
        text(125, 535, f"{data_sectors:,} sectors × 4 KiB = {data_pages:,} pages × 256 B", 16),
        text(125, 565, f"Each page: {RECORDS_PER_PAGE} × {RECORD_SIZE}-byte FlashLogRecord_t", 16),
        text(80, 625, "One data-page detail", 22, "bold"),
        text(100, 665, fmt_address(DATA_START), 14),
        text(100, 695, "page start", 14),
        rect(255, 650, 220, 58, "#E6A23C"),
        rect(475, 650, 220, 58, "#80CFA0"),
        rect(695, 650, 220, 58, "#80CFA0"),
        rect(915, 650, 220, 58, "#80CFA0"),
        text(270, 684, "record 0", 15, "bold"),
        text(490, 684, "record 1", 15, "bold"),
        text(710, 684, "…", 20, "bold"),
        text(930, 684, "record 7", 15, "bold"),
        text(80, 755, "Marker pages consume a full 256-byte page; the marker itself occupies only the first 32 bytes.", 16),
        text(80, 782, "Normal pages contain eight packed FlashLogRecord_t records (256 bytes total).", 16),
        text(80, 810, f"Current view: {used_pages:,} used data pages ({data_used:,} B), {data_free:,} B free, {max_records:,} record slots total.", 16),
        '</svg>',
    ]
    return "\n".join(part for part in svg if part)


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--write-pointer", type=lambda value: int(value, 0), default=DATA_START,
                        help="Header write pointer, e.g. 0x11000 (default: data start)")
    parser.add_argument("--flight-count", type=int, default=0,
                        help="Flight count to show in the diagram (default: 0)")
    parser.add_argument("--output", type=Path, default=Path("flash_memory_map.svg"),
                        help="Output SVG path")
    args = parser.parse_args()
    args.output.write_text(make_svg(args.write_pointer, args.flight_count), encoding="utf-8")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
