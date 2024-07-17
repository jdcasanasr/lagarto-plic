//**********************************************************************//
//  Proyecto:       SoC_AXI_Lagarto_I                                   //
//  Archivo:        lagarto_plic_gateway.v                              //
//  Organización:   Instituto Politécnico Nacional                      //
//  Autor(es):      Daniel Casañas                                      //
//  Supervisor:     Dr. Marco Antonio Ramírez Salinas                   //
//  E-mail:         lagarto@cic.ipn.mx                                  //
//  Referencias:    https://github.com/riscv/riscv-plic-spec            //
//**********************************************************************//

module lagarto_plic_gateway
(
    input   logic   interrupt_signal_i,
    input   logic   interrupt_complete_i,

    output  logic   interrupt_request_o
);

    always_latch
        if (interrupt_complete_i)
            interrupt_request_o = interrupt_signal_i;

endmodule