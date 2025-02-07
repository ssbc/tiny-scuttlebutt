# Makefile

# the LVGL-specific code is put in this separate directory (not under src)
# such that the arduino-cli only parses it when TINYSSB_BOARD_TDECK is
# defined (otherwise the stray *.c image files would be considered)

# convert all image *.c files into *.h files to be #included

# this conversion replaces the C-specific "designated initializers"
# into an initialization that compiles in C++

all:
	for i in *.c; do \
	  awk -f c_map_to_h.awk < $${i} > $${i%%.c}.h; \
	done

# eof
