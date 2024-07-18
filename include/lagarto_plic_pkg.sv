package lagarto_plic_pkg;

    import riscv_privileged_pkg :: MXLEN;

    localparam INTERRUPT_ENABLE_MASK = 32'h3;

    typedef enum logic [MXLEN - 1:0]
    {
        NO_INTERRUPT_PRIORITY   = 'd0,
        JTAG0_PRIORITY          = 'd1,
        JTAG1_PRIORITY          = 'd2
    } interrupt_priority_t;

    typedef enum logic [MXLEN - 1:0]
    {
        NO_INTERRUPT_ID = 'd0,
        JTAG0_ID        = 'd1,
        JTAG1_ID        = 'd2
    } interrupt_id_t;

endpackage