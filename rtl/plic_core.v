//**********************************************************************//
//  Proyecto:       SoC_AXI_Lagarto_I                                   //
//  Archivo:        plic_core.v                                         //
//  Organización:   Instituto Politécnico Nacional                      //
//  Autor(es):      Daniel Casañas                                      //
//  Supervisor:     Dr. Marco Antonio Ramírez Salinas                   //
//  E-mail:         lagarto@cic.ipn.mx                                  //
//  Referencias:    https://github.com/riscv/riscv-plic-spec            //
//**********************************************************************//

`include "../include/lagarto_hun_plic.vh"

module plic_core
#(
    parameter N_INTERRUPT_SOURCES       = 32
)
(
    // Source-side signals.
    input       [N_INTERRUPT_SOURCES - 1:0]     interrupt_source_signal_i,
    input       [N_INTERRUPT_SOURCES - 1:0]     interrupt_source_enable_i,
    input       [31:0]                          interrupt_source_priority_i     [N_INTERRUPT_SOURCES - 1:0],
    input       [31:0]                          interrupt_source_id_i           [N_INTERRUPT_SOURCES - 1:0],
    // ToDo: PC para la rutina de interrupción pertinente.

    // Target-side signals.
    input                                       interrupt_target_ready_i,
    input       [31:0]                          interrupt_target_claim_i,
    input       [31:0]                          interrupt_target_priority_threshold_i,
    

    output reg                                  interrupt_target_notification_o,
    output reg  [31:0]                          interrupt_target_id_o
);

    wire    [N_INTERRUPT_SOURCES - 1:0] interrupt_pending_w;
    wire    [31:0]                      greatest_interrupt_source_priority_w    [N_INTERRUPT_SOURCES - 1:0];
    wire    [31:0]                      greatest_interrupt_source_priority_id_w [N_INTERRUPT_SOURCES - 1:0];

    always @ (*)
        if (greatest_interrupt_source_priority_w[31] > interrupt_target_priority_threshold_i)
            interrupt_target_notification_o = 1'b1;

        else if (interrupt_target_claim_i)
            interrupt_target_notification_o = 1'b0;

        else
            interrupt_target_notification_o = interrupt_target_notification_o;

    always @ (*)
        if (interrupt_target_claim_i)
            interrupt_target_id_o   = greatest_interrupt_source_priority_id_w[31];

        else interrupt_target_id_o  = `ID_NO_INTERRUPT;

    generate
        genvar i;

        for(i = 0; i < N_INTERRUPT_SOURCES; i = i + 1)
            begin   : generate_gateways
                plic_gateway peripheral_gateway
                (
                    .interrupt_source_signal_i      (interrupt_source_signal_i[i]),
                    .interrupt_target_ready_i       (interrupt_target_ready_i),

                    .interrupt_source_request_o     (interrupt_pending_w[i])
                );
            end
    endgenerate

    plic_cell plic_cell_0
    (
        .interrupt_enable_i                     (interrupt_source_enable_i[0]),
        .interrupt_pending_i                    (interrupt_pending_w[0]),

        .interrupt_source_a_priority_i          (`PRIORITY_NEVER_INTERRUPT),
        .interrupt_source_a_id_i                (`ID_NO_INTERRUPT),

        .interrupt_source_b_priority_i          (interrupt_source_priority_i[0]),
        .interrupt_source_b_id_i                (interrupt_source_id_i[0]),

        .interrupt_source_maximum_priority_o    (greatest_interrupt_source_priority_w[0]),
        .interrupt_source_maximum_priority_id_o (greatest_interrupt_source_priority_id_w[0])
    );

    generate
        genvar j;

        for(j = 1; j < N_INTERRUPT_SOURCES; j = j + 1)
            begin   : generate_cells
                plic_cell plic_cell_i
                (
                    .interrupt_enable_i                     (interrupt_source_enable_i[j]),
                    .interrupt_pending_i                    (interrupt_pending_w[j]),

                    .interrupt_source_a_priority_i          (greatest_interrupt_source_priority_w[j - 1]),
                    .interrupt_source_a_id_i                (greatest_interrupt_source_priority_id_w[j - 1]),

                    .interrupt_source_b_priority_i          (interrupt_source_priority_i[j]),
                    .interrupt_source_b_id_i                (interrupt_source_id_i[j]),

                    .interrupt_source_maximum_priority_o    (greatest_interrupt_source_priority_w[j]),
                    .interrupt_source_maximum_priority_id_o (greatest_interrupt_source_priority_id_w[j])
                );
            end
    endgenerate

endmodule