//**********************************************************************//
//  Proyecto:       SoC_AXI_Lagarto_I                                   //
//  Archivo:        lagarto_plic.sv                                     //
//  Organización:   Instituto Politécnico Nacional                      //
//  Autor(es):      Daniel Casañas                                      //
//  Supervisor:     Dr. Marco Antonio Ramírez Salinas                   //
//  E-mail:         lagarto@cic.ipn.mx                                  //
//  Referencias:    https://github.com/riscv/riscv-plic-spec            //
//**********************************************************************//

import lagarto_plic_pkg     :: * ;
import riscv_privileged_pkg :: MXLEN;

module lagarto_plic
#(
    parameter PLIC_REGISTER_LENGTH          = 32
    parameter NUMBER_OF_INTERRUPT_SOURCES   = 32
)
(
    // Source-Side Interface.
    input logic     [NUMBER_OF_INTERRUPT_SOURCES - 1:0]     interrupt_signal_i,

    // Target-Side Interface.
    input logic                                             interrupt_claim_i,
    input logic                                             interrupt_complete_i,

    output logic                                            interrupt_notification_o,
    output logic    [MXLEN - 1:0]                           interrupt_id_o
);

    // Internal Signals & Buses.
    logic [NUMBER_OF_INTERRUPT_SOURCES - 1:0] interrupt_request_w;

    // Sub-Module Instances.
    genvar i;

    generate
        for (i = 0; i < NUMBER_OF_INTERRUPT_SOURCES; i += 1)
            begin   : lagarto_plic_gateway_instantiation
                lagarto_plic_gateway lagarto_plic_gateway_instance
                (
                    .interrupt_signal_i     (interrupt_signal_i),
                    .interrupt_complete_i   (interrupt_complete_i),

                    .interrupt_request_o    (interrupt_request_w)
                );
            end     : lagarto_plic_gateway_instantiation
    endgenerate

endmodule