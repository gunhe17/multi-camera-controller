import struct
import io
from PIL import Image
from etw import ETW

def is_valid_jpeg(data: bytes) -> bool:
    return data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9'

def extract_jpeg_frames(buffer: bytes, packets: list[dict]) -> list:
    frames = []
    for pkt in packets:
        offset = pkt['offset']
        length = pkt['length']
        if offset + length > len(buffer) or length == 0:
            continue
        segment = buffer[offset:offset + length]
        if is_valid_jpeg(segment):
            try:
                Image.open(io.BytesIO(segment)).verify()
                frames.append(segment)
            except:
                continue
    return frames

def parse_iso_packets(raw: bytes, count: int) -> list:
    packets = []
    for i in range(count):
        base = i * 12
        if base + 12 <= len(raw):
            offset, length, status = struct.unpack_from("<III", raw, base)
            packets.append({"offset": offset, "length": length})
    return packets

def analyze_etl(etl_path: str):
    frame_count = 0

    def on_event(event):
        nonlocal frame_count

        if "URB_FUNCTION_ISOCH_TRANSFER" not in event["event_name"]:
            return

        # TransferBufferLength
        urb_len = int(event.get("fid_URB_TransferBufferLength", "0"), 16)
        urb_buf = event.get("fid_URB_TransferBuffer", "")
        urb_num_pkts = int(event.get("fid_UCX_URB_NumberOfPackets", "0"), 16)
        iso_pkt_raw = event.get("fid_UCX_URB_ISO_PACKETS", "")

        if urb_len == 0 or urb_buf == "" or urb_num_pkts == 0 or iso_pkt_raw == "":
            return

        try:
            raw_bytes = bytes.fromhex(urb_buf.replace("0x", "").replace(":", "").replace(" ", ""))
            packet_list = parse_iso_packets(bytes.fromhex(iso_pkt_raw.replace("{", "").replace("}", "").replace(":", "").replace(";", "").replace(" ", "")), urb_num_pkts)
            frames = extract_jpeg_frames(raw_bytes, packet_list)
            frame_count += len(frames)
        except:
            return

    etw = ETW(providers=[{
        "guid": "Microsoft-Windows-USB-UCX",
        "any": 0xFFFFFFFFFFFFFFFF,
        "all": 0
    }], file=etl_path, event_callback=on_event)

    etw.process()
    print(f"Total MJPEG Frames: {frame_count}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python analyze_mjpeg_etl.py usbtrace.etl")
    else:
        analyze_etl(sys.argv[1])
