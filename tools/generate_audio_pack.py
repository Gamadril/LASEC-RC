import os
import struct
import shutil
import csv
Import("env")

board = env.BoardConfig()
mcu = board.get("build.mcu", "esp32")
port = "/dev/ttyACM0"

# Helper to read partition table and obtain the offset for a given partition name
def _get_partition_offset(env, name):
    csv_path = env.GetProjectOption("board_build.partitions")
    if not os.path.isfile(csv_path):
        raise RuntimeError(f"Partition table {csv_path} not found")
    with open(csv_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if parts[0] == name:
                offset = parts[3]
                # Ensure hex format
                if not offset.lower().startswith("0x"):
                    offset = hex(int(offset))
                return offset
    raise RuntimeError(f"Partition '{name}' not found in {csv_path}")

def generate_audio_pack(source, target, env):
    print("Generating audio pack...")
    
    data_dir = os.path.join(env.get("PROJECT_DIR"), "sounds")
    spiffs_dir = os.path.join(env.get("PROJECT_DIR"), "data")
    output_bin = os.path.join(env.get("PROJECT_DIR"), "build", "audio.bin")
    
    if not os.path.exists(data_dir):
        print(f"Error: {data_dir} does not exist")
        return

    # Ensure spiffs_dir is clean and exists
    if os.path.exists(spiffs_dir):
        shutil.rmtree(spiffs_dir)
    os.makedirs(spiffs_dir)

    # Ensure data_spiffs root exists (for other files if any)
    if not os.path.exists(os.path.join(env.get("PROJECT_DIR"), "data_spiffs")):
         os.makedirs(os.path.join(env.get("PROJECT_DIR"), "data_spiffs"))

    offset = 0
    with open(output_bin, "wb") as outfile:
        for filename in sorted(os.listdir(data_dir)):
            if not filename.endswith(".wav"):
                continue
                
            filepath = os.path.join(data_dir, filename)
            with open(filepath, "rb") as infile:
                in_data = infile.read()
                in_size = len(in_data)
                outfile.write(in_data)
                
                # Write metadata file to SPIFFS
                # Format: offset (4 bytes), length (4 bytes)
                meta_path = os.path.join(spiffs_dir, filename)
                with open(meta_path, "wb") as metafile:
                    metafile.write(struct.pack("<II", offset, in_size))
                
                print(f"Packed {filename}: offset={offset}, size={in_size}")
                offset += in_size

    print(f"Audio pack generated: {output_bin} ({offset} bytes)")

# Use the specific hook requested by the user for building SPIFFS image
env.AddPreAction("$BUILD_DIR/spiffs.bin", generate_audio_pack)

# upload audio.bin
def upload_audio_bin(source, target, env):
    audio_path = os.path.join(env.get("PROJECT_DIR"), "build", "audio.bin")
    flash_addr = _get_partition_offset(env, "audio_data")
    print("Uploading audio.bin ...")
    env.Execute("$PYTHONEXE $UPLOADER --chip %s --port %s --baud $UPLOAD_SPEED write_flash %s %s" % (mcu, port, flash_addr, audio_path))

# Run after the normal uploadfs finishes
env.AddPostAction("uploadfs", upload_audio_bin)
    
def dump_flash(source, target, env):
    print("Dumping flash ...")  
    env.Execute("$PYTHONEXE $UPLOADER --chip %s --port %s --baud $UPLOAD_SPEED read_flash 0 ALL flash_dump.bin" % (mcu, port))

# Add custom target
env.AddCustomTarget(
    "generate_audio_pack",
    None,
    generate_audio_pack,
    title="Generate Audio Pack",
    description="Generate audio.bin and SPIFFS metadata"
)

env.AddCustomTarget(
    "upload_audio_bin",
    None,
    upload_audio_bin,
    title="Upload Audio Pack",
    description="Upload audio.bin to device"
)

env.AddCustomTarget(
    "dump_flash",
    None,
    dump_flash,
    title="Dump flash",
    description="Dump flash content to flash_dump.bin"
)

print("Audio Pack Script Loaded")