package lagarto_plic_pkg;

    parameter INTERRUPT_ENABLE_MASK = 32'h0;
    parameter NO_INTERRUPT          = '0;

    typedef enum
    {
        JTAG_0,
        JTAG_1,
    } interrupt_source_id_t;

endpackage