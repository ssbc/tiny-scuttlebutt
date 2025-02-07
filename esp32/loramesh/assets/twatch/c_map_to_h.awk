/const lv_img/ { found=1; nm=$3; print("/*"); }

/header.cf/ { cf=$3; }

/header.w/  { wi=$3; }

/header.h/  { hi=$3; }

/data_size/ { ds=substr($0, 16); }

/data[ ]/   { da=$3; }

/\};/ { print($0); if (found == 1) print("*/\n"); next;}

/.*/ { print($0); }

END {
    print("const lv_img_dsc_t " nm " = {");
    print("#if LV_BIG_ENDIAN_SYSTEM");
    print("  { " hi " " wi " 0, 0, " cf " },");
    print("#else");
    print("  { " cf " 0, 0, " wi " " hi " },");
    print("#endif");
    print("  " ds " " da);
    print("};");
    print("\n// eof");
}
