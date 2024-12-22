//**********************************************************************//
//  Proyecto:       SoC_AXI_Lagarto_I                                   //
//  Archivo:        lagarto_plic.sv                                     //
//  Organización:   Instituto Politécnico Nacional                      //
//  Autor(es):      Daniel Casañas                                      //
//  Supervisor:     Dr. Marco Antonio Ramírez Salinas                   //
//  E-mail:         lagarto@cic.ipn.mx                                  //
//  Referencias:    https://github.com/riscv/riscv-plic-spec            //
//**********************************************************************//

import lagarto_plic_pkg     :: *;
import riscv_privileged_pkg :: MXLEN;

module lagarto_plic
#(
    parameter PLIC_REGISTER_LENGTH          = 32,
    parameter NUMBER_OF_INTERRUPT_SOURCES   = 2
)
(
    // Source-Side Interface.
    input logic             [NUMBER_OF_INTERRUPT_SOURCES - 1:0] interrupt_signal_i,

    // Target-Side Interface.
    input logic                                                 interrupt_claim_complete_i,

    output logic                                                interrupt_notification_o,
    output interrupt_id_t                                       interrupt_id_o
);

    // Internal Signals & Buses.
    logic                   [NUMBER_OF_INTERRUPT_SOURCES - 1:0] interrupt_enable_w;
    logic                   [NUMBER_OF_INTERRUPT_SOURCES - 1:0] interrupt_request_w;

    interrupt_priority_t    [NUMBER_OF_INTERRUPT_SOURCES - 1:0] maximum_priority_w;
    interrupt_id_t          [NUMBER_OF_INTERRUPT_SOURCES - 1:0] maximum_id_w;

    assign interrupt_enable_w = INTERRUPT_ENABLE_MASK;

    // Drive Output Ports.
    // See If There's At Least One Interrupt Enable/Request Match.
    assign interrupt_notification_o = |(interrupt_enable_w & interrupt_request_w);
    assign interrupt_id_o           = maximum_id_w[NUMBER_OF_INTERRUPT_SOURCES - 1];

    // Sub-Module Instances.
    genvar i;

    generate
        for (i = 0; i < NUMBER_OF_INTERRUPT_SOURCES; i += 1)
            begin   : lagarto_plic_gateway_instantiation
                lagarto_plic_gateway lagarto_plic_gateway_instance
                (
                    .interrupt_signal_i             (interrupt_signal_i[i]),
                    .interrupt_claim_complete_i     (interrupt_claim_complete_i),

                    .interrupt_request_o            (interrupt_request_w[i])
                );
            end     : lagarto_plic_gateway_instantiation
    endgenerate

    // Generate Multiplexer Instances By Hand.
    lagarto_plic_multiplexer jtag0_multiplexer
    (
        .source_interrupt_pending_i (interrupt_request_w[0]),
        .interrupt_enable_i         (interrupt_enable_w[0]),

        .interrupt_priority_a_i     (JTAG0_PRIORITY),
        .interrupt_id_a_i           (JTAG0_ID),

        .interrupt_priority_b_i     (NO_INTERRUPT_PRIORITY),
        .interrupt_id_b_i           (NO_INTERRUPT_ID),

        .maximum_priority_o         (maximum_priority_w[0]),
        .maximum_id_o               (maximum_id_w[0])
    );

    lagarto_plic_multiplexer jtag1_multiplexer
    (
        .source_interrupt_pending_i (interrupt_request_w[1]),
        .interrupt_enable_i         (interrupt_enable_w[1]),

        .interrupt_priority_a_i     (JTAG1_PRIORITY),
        .interrupt_id_a_i           (JTAG1_ID),

        .interrupt_priority_b_i     (maximum_priority_w[0]),
        .interrupt_id_b_i           (maximum_id_w[0]),

        .maximum_priority_o         (maximum_priority_w[1]),
        .maximum_id_o               (maximum_id_w[1])
    );

endmodule