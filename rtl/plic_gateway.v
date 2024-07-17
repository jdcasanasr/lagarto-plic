//**********************************************************************//
//  Proyecto:       SoC_AXI_Lagarto_I                                   //
//  Archivo:        plic_gateway_level.v                                //
//  Organización:   Instituto Politécnico Nacional                      //
//  Autor(es):      Daniel Casañas                                      //
//  Supervisor:     Dr. Marco Antonio Ramírez Salinas                   //
//  E-mail:         lagarto@cic.ipn.mx                                  //
//  Referencias:    https://github.com/riscv/riscv-plic-spec            //
//**********************************************************************//

module plic_gateway
(
    input       interrupt_source_signal_i,
    input       interrupt_target_ready_i,

    // The output is latched when the conditions are met,
    // even if the input changes before the interrupt is
    // serviced.
    output reg  interrupt_source_request_o
);

    always @ (*)
        if (interrupt_target_ready_i)
            interrupt_source_request_o = interrupt_source_signal_i;

endmodule