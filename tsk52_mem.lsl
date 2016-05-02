#include "tsk52.lsl"

derivative system
{
    core sw
    {
        architecture = __SW_ARCH;
    }

    memory iram
    {
        mau = 8;
        type = ram;
        size = 256;
        map(dest=bus:sw:idata_bus, src_offset=0x0, dest_offset=0x0, size=256);
    }

    memory xram
    {
        mau = 8;
        type = ram;
        size = 64k;
        map(dest=bus:sw:xdata_bus, src_offset=0x0, dest_offset=0x0, size=64k);
    }

    memory xrom
    {
        mau = 8;
        type = rom;
        size = 64k;
        map(dest=bus:sw:code_bus, src_offset=0x0, dest_offset=0x0, size=64k);
    }





}
